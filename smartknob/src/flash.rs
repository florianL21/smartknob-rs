extern crate alloc;

use crate::config::LogChannelToggles;
use crate::motor_control::MotorAlignment;
use alloc::format;
use ekv::flash::{self, PageID};
use ekv::{config, Database, ReadError};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::{self, FlashRegion};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;
use log::{info, warn};
use postcard::experimental::max_size::MaxSize;
use static_cell::make_static;
use thiserror::Error;
use ufmt::uDebug;

pub type FlashType<'a> =
    Database<PersistentStorage<FlashRegion<'a, FlashStorage<'a>>>, NoopRawMutex>;
pub type FlashErrorType = esp_bootloader_esp_idf::partitions::Error;

// Workaround for alignment requirements.
#[repr(C, align(4))]
struct AlignedBuf<const N: usize>([u8; N]);

pub struct PersistentStorage<T: NorFlash + ReadNorFlash> {
    start: usize,
    flash: T,
}

#[derive(Copy, Clone)]
pub enum FlashKeys {
    Test,
    LogChannels,
    MotorAlignment,
}

impl<'a> FlashKeys {
    pub fn key(&'a self) -> [u8; 1] {
        [*self as u8]
    }
}

#[derive(Error, Debug)]
pub enum FlashError {
    #[error("Flash read failed: {0:#?}")]
    FlashReadError(ekv::ReadError<FlashErrorType>),
    #[error("Flash write failed: {0:#?}")]
    FlashWriteError(ekv::WriteError<FlashErrorType>),
    #[error("Flash format failed: {0:#?}")]
    FlashFormatError(ekv::FormatError<FlashErrorType>),
    #[error("Flash commit failed: {0:#?}")]
    FlashCommitError(ekv::CommitError<FlashErrorType>),
    #[error("Postcard error: {0:#?}")]
    PostcardError(#[from] postcard::Error),
}

impl From<ekv::ReadError<FlashErrorType>> for FlashError {
    fn from(value: ekv::ReadError<FlashErrorType>) -> Self {
        FlashError::FlashReadError(value.into())
    }
}

impl From<ekv::WriteError<FlashErrorType>> for FlashError {
    fn from(value: ekv::WriteError<FlashErrorType>) -> Self {
        FlashError::FlashWriteError(value.into())
    }
}

impl From<ekv::FormatError<FlashErrorType>> for FlashError {
    fn from(value: ekv::FormatError<FlashErrorType>) -> Self {
        FlashError::FlashFormatError(value.into())
    }
}

impl From<ekv::CommitError<FlashErrorType>> for FlashError {
    fn from(value: ekv::CommitError<FlashErrorType>) -> Self {
        FlashError::FlashCommitError(value.into())
    }
}

impl uDebug for FlashError {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        f.write_str(format!("{self:?}").as_str())
    }
}

impl<T: NorFlash + ReadNorFlash> flash::Flash for PersistentStorage<T> {
    type Error = T::Error;

    fn page_count(&self) -> usize {
        config::MAX_PAGE_COUNT
    }

    async fn erase(
        &mut self,
        page_id: PageID,
    ) -> Result<(), <PersistentStorage<T> as flash::Flash>::Error> {
        let from = (self.start + page_id.index() * config::PAGE_SIZE) as u32;
        let to = (self.start + page_id.index() * config::PAGE_SIZE + config::PAGE_SIZE) as u32;
        self.flash.erase(from, to)
    }

    async fn read(
        &mut self,
        page_id: PageID,
        offset: usize,
        data: &mut [u8],
    ) -> Result<(), <PersistentStorage<T> as flash::Flash>::Error> {
        let address = self.start + page_id.index() * config::PAGE_SIZE + offset;
        let mut buf = AlignedBuf([0; config::PAGE_SIZE]);
        self.flash.read(address as u32, &mut buf.0[..data.len()])?;
        data.copy_from_slice(&buf.0[..data.len()]);
        Ok(())
    }

    async fn write(
        &mut self,
        page_id: PageID,
        offset: usize,
        data: &[u8],
    ) -> Result<(), <PersistentStorage<T> as flash::Flash>::Error> {
        let address = self.start + page_id.index() * config::PAGE_SIZE + offset;
        let mut buf = AlignedBuf([0; config::PAGE_SIZE]);
        buf.0[..data.len()].copy_from_slice(data);
        self.flash.write(address as u32, &buf.0[..data.len()])
    }
}

pub struct FlashHandler {
    flash: FlashType<'static>,
}

#[derive(Debug, Default)]
pub struct RestoredState {
    pub log_channels: LogChannelToggles,
    pub motor_alignment: Option<MotorAlignment>,
}

impl FlashHandler {
    pub async fn new(flash: FLASH<'static>) -> Self {
        let flash = make_static!(FlashStorage::new(flash));
        let pt_mem = make_static!([0u8; partitions::PARTITION_TABLE_MAX_LEN]);
        let pt = partitions::read_partition_table(flash, pt_mem).unwrap();
        let fat = make_static!(pt
            .find_partition(partitions::PartitionType::Data(
                partitions::DataPartitionSubType::Fat,
            ))
            .expect("Failed to search for partitions")
            .expect("Could not find a data:fat partition"));
        let offset = fat.offset();
        info!("Storing data into partition with offset: {offset}");
        let fat_partition = fat.as_embedded_storage(flash);

        let flash = PersistentStorage {
            flash: fat_partition,
            start: 0,
        };

        let flash = FlashType::new(flash, ekv::Config::default());
        if flash.mount().await.is_ok() {
            info!("Flash mounted successfully");
        } else {
            warn!("Failed to mount flash. Assuming first boot, formatting...");
            flash.format().await.expect("Failed to format flash");
        }

        Self { flash }
    }

    /// Restore the system state from flash. Intended to be called during startup
    pub async fn restore(&self) -> Result<RestoredState, FlashError> {
        let rt = self.flash.read_transaction().await;

        // Get the largest POSTCARD_MAX_SIZE to allocate a buffer
        const MAX_BUF_SIZE: usize = [
            LogChannelToggles::POSTCARD_MAX_SIZE,
            MotorAlignment::POSTCARD_MAX_SIZE,
        ][(LogChannelToggles::POSTCARD_MAX_SIZE < MotorAlignment::POSTCARD_MAX_SIZE) as usize];

        // Restore log toggle configuration
        let mut buffer = [0u8; MAX_BUF_SIZE];
        let size = rt.read(&FlashKeys::LogChannels.key(), &mut buffer).await?;
        let log_toggles: LogChannelToggles = postcard::from_bytes(&buffer[..size])?;

        // Restore motor alignment state
        let motor_alignment = match rt.read(&FlashKeys::MotorAlignment.key(), &mut buffer).await {
            Ok(size) => {
                let motor_alignment: MotorAlignment = postcard::from_bytes(&buffer[..size])?;
                Some(motor_alignment)
            }
            Err(ReadError::KeyNotFound) => {
                info!("No motor alignment state found in flash");
                None
            }
            Err(e) => {
                return Err(FlashError::from(e));
            }
        };

        Ok(RestoredState {
            log_channels: log_toggles,
            motor_alignment: motor_alignment,
        })
    }

    /// After all init operations are finished on the flash this method can be used to get the underlying struct back
    pub fn eject(self) -> FlashType<'static> {
        self.flash
    }
}
