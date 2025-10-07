extern crate alloc;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;
use ekv::flash::{self, PageID};
use ekv::{config, Database, ReadError};
use embassy_executor::task;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::{self, FlashRegion};
use esp_hal::peripherals::FLASH;
use esp_hal::system::{Cpu, CpuControl};
use esp_storage::FlashStorage;
use log::info;
use static_cell::make_static;

pub type FlashType<'a> =
    Database<PersistentStorage<FlashRegion<'a, FlashStorage<'a>>>, NoopRawMutex>;

// Workaround for alignment requirements.
#[repr(C, align(4))]
struct AlignedBuf<const N: usize>([u8; N]);

pub struct PersistentStorage<T: NorFlash + ReadNorFlash> {
    start: usize,
    flash: T,
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

// pub fn flash_init(flash: FLASH) -> FlashType {
//     let flash = make_static!(FlashStorage::new(flash));
//     let pt_mem = make_static!([0u8; partitions::PARTITION_TABLE_MAX_LEN]);
//     let pt = partitions::read_partition_table(flash, pt_mem).unwrap();
//     let fat = make_static!(pt
//         .find_partition(partitions::PartitionType::Data(
//             partitions::DataPartitionSubType::Fat,
//         ))
//         .expect("Failed to search for partitions")
//         .expect("Could not find a data:littlefs partition"));
//     let offset = fat.offset();
//     info!("Storing data into partition with offset: {offset}");
//     let fat_partition = fat.as_embedded_storage(flash);

//     let flash = PersistentStorage {
//         flash: fat_partition,
//         start: 0,
//     };

//     FlashType::new(flash, ekv::Config::default())
// }
