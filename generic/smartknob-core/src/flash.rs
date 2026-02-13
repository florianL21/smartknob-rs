extern crate alloc;

use alloc::format;
use core::fmt::Debug;
use ekv::flash::{self, PageID};
use ekv::{Database, ReadError, config};
use embassy_futures::select;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use log::{error, info};
use postcard::Deserializer;
use postcard::de_flavors::Slice;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use ufmt::uDebug;

use crate::haptic_core::CalibrationData;
use crate::system_settings::StoreSignals;
use crate::system_settings::log_toggles::LogChannelToggles;

pub type FlashType<'a, Flash> = Database<PersistentStorage<Flash>, NoopRawMutex>;
pub type FlashErrorType<Flash> = FlashError<<Flash as ErrorType>::Error>;

const fn max(a: usize, b: usize) -> usize {
    [a, b][(a < b) as usize]
}

const MAX_BUFFER_SIZE: usize = max(
    CalibrationData::POSTCARD_MAX_SIZE,
    LogChannelToggles::POSTCARD_MAX_SIZE,
);

// Workaround for alignment requirements.
#[repr(C, align(4))]
struct AlignedBuf<const N: usize>([u8; N]);

pub struct PersistentStorage<T: NorFlash + ReadNorFlash> {
    start: usize,
    flash: T,
}

impl<T: NorFlash + ReadNorFlash> PersistentStorage<T> {
    pub fn new(start: usize, flash: T) -> Self {
        Self { start, flash }
    }
}

#[derive(Copy, Clone)]
pub enum FlashKeys {
    LogChannels,
    MotorAlignment,
}

pub struct RestoredState {
    pub haptic_core: Option<CalibrationData>,
    pub log_toggles: Option<LogChannelToggles>,
}

impl<'a> FlashKeys {
    pub fn key(&'a self) -> [u8; 1] {
        [*self as u8]
    }
}

#[derive(Error, Debug)]
pub enum FlashError<FE> {
    #[error("Flash read failed: {0:#?}")]
    FlashReadError(ekv::ReadError<FE>),
    #[error("Flash write failed: {0:#?}")]
    FlashWriteError(ekv::WriteError<FE>),
    #[error("Flash format failed: {0:#?}")]
    FlashFormatError(ekv::FormatError<FE>),
    #[error("Flash commit failed: {0:#?}")]
    FlashCommitError(ekv::CommitError<FE>),
    #[error("Postcard error: {0:#?}")]
    PostcardError(#[from] postcard::Error),
}

impl<FE> From<ekv::ReadError<FE>> for FlashError<FE> {
    fn from(value: ekv::ReadError<FE>) -> Self {
        FlashError::FlashReadError(value)
    }
}

impl<FE> From<ekv::WriteError<FE>> for FlashError<FE> {
    fn from(value: ekv::WriteError<FE>) -> Self {
        FlashError::FlashWriteError(value)
    }
}

impl<FE> From<ekv::FormatError<FE>> for FlashError<FE> {
    fn from(value: ekv::FormatError<FE>) -> Self {
        FlashError::FlashFormatError(value)
    }
}

impl<FE> From<ekv::CommitError<FE>> for FlashError<FE> {
    fn from(value: ekv::CommitError<FE>) -> Self {
        FlashError::FlashCommitError(value)
    }
}

impl<FE: Debug> uDebug for FlashError<FE> {
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

pub trait FlashHandling<Flash: NorFlash> {
    fn get_flash(&self) -> &FlashType<'static, Flash>;

    fn load<'a, T: Deserialize<'a>>(
        &self,
        key: FlashKeys,
        buffer: &'a mut [u8],
    ) -> impl core::future::Future<Output = Result<Option<T>, FlashErrorType<Flash>>> {
        async move {
            let rt = self.get_flash().read_transaction().await;
            match rt.read(&key.key(), buffer).await {
                Err(ReadError::KeyNotFound) => return Ok(None),
                Err(e) => return Err(FlashError::from(e)),
                Ok(_) => {}
            }
            let slice = Slice::new(buffer);
            let mut deserializer = Deserializer::from_flavor(slice);
            let data = T::deserialize(&mut deserializer)?;
            Ok(Some(data))
        }
    }

    fn store<T: Serialize, const M: usize>(
        &self,
        key: &FlashKeys,
        value: &T,
    ) -> impl core::future::Future<Output = Result<(), FlashErrorType<Flash>>> {
        async {
            let mut wt = self.get_flash().write_transaction().await;
            let mut buffer = [0u8; M];
            postcard::to_slice(value, &mut buffer)?;

            wt.write(&key.key(), &buffer)
                .await
                .map_err(FlashError::from)?;
            wt.commit().await.map_err(FlashError::from)?;
            Ok(())
        }
    }

    fn format(&self) -> impl core::future::Future<Output = Result<(), FlashErrorType<Flash>>> {
        async {
            self.get_flash().format().await?;
            Ok(())
        }
    }

    fn restore(&self) -> impl core::future::Future<Output = RestoredState> {
        async {
            // restore motor calibration
            let mut buffer = [0u8; MAX_BUFFER_SIZE];
            let haptic_core = match self.load(FlashKeys::MotorAlignment, &mut buffer).await {
                Ok(cal) => cal,
                Err(e) => {
                    error!("Failed to read cal data from flash: {e}");
                    None
                }
            };
            let mut buffer = [0u8; MAX_BUFFER_SIZE];
            // restore log toggles
            let log_toggles = match self.load(FlashKeys::LogChannels, &mut buffer).await {
                Ok(log_toggles) => log_toggles,
                Err(e) => {
                    error!("Failed to read log toggle data from flash: {e}");
                    None
                }
            };
            RestoredState {
                haptic_core,
                log_toggles,
            }
        }
    }

    fn run<M: RawMutex>(
        &self,
        restore_signals: &'static StoreSignals<M>,
    ) -> impl core::future::Future<Output = ()> {
        async {
            match select::select(
                restore_signals.haptic_core.wait(),
                restore_signals.log_toggles.wait(),
            )
            .await
            {
                select::Either::First(data) => {
                    if let Err(e) = self
                        .store::<_, { CalibrationData::POSTCARD_MAX_SIZE }>(
                            &FlashKeys::MotorAlignment,
                            &data,
                        )
                        .await
                    {
                        error!("Failed to store calibration to flash: {e}");
                    } else {
                        info!("Stored successfully!")
                    }
                }
                select::Either::Second(cal) => {
                    if let Err(e) = self
                        .store::<_, { LogChannelToggles::POSTCARD_MAX_SIZE }>(
                            &FlashKeys::LogChannels,
                            &cal,
                        )
                        .await
                    {
                        error!("Failed to store log channels to flash: {e}");
                    } else {
                        info!("Stored successfully!")
                    }
                }
            }
        }
    }
}
