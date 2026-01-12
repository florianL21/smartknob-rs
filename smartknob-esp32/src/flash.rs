extern crate alloc;

use esp_bootloader_esp_idf::partitions::{self, FlashRegion, PartitionEntry, PartitionType};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;
use log::{info, warn};
use smartknob_core::flash::{FlashError, FlashHandling, FlashType, PersistentStorage};
use static_cell::StaticCell;

pub type ESPFlashError = FlashError<partitions::Error>;
type ESPFlashType = FlashRegion<'static, FlashStorage<'static>>;

pub struct FlashHandler {
    flash: FlashType<'static, ESPFlashType>,
}

impl FlashHandling<ESPFlashType> for FlashHandler {
    fn get_flash(&self) -> &FlashType<'static, ESPFlashType> {
        &self.flash
    }
}

impl FlashHandler {
    pub async fn new(flash: FLASH<'static>, partition: PartitionType) -> Self {
        static FLASH: StaticCell<FlashStorage> = StaticCell::new();
        let flash = FLASH.init(FlashStorage::new(flash).multicore_auto_park());
        static PT_MEM: StaticCell<[u8; partitions::PARTITION_TABLE_MAX_LEN]> = StaticCell::new();
        let pt_mem = PT_MEM.init([0u8; partitions::PARTITION_TABLE_MAX_LEN]);
        let pt = partitions::read_partition_table(flash, pt_mem).unwrap();
        static FAT: StaticCell<PartitionEntry> = StaticCell::new();
        let fat = FAT.init(
            pt.find_partition(partition)
                .expect("Failed to search for partitions")
                .expect("Could not find the specified partition"),
        );
        let offset = fat.offset();
        info!("Storing data into partition with offset: {offset}");
        let fat_partition = fat.as_embedded_storage(flash);

        let flash = PersistentStorage::new(0, fat_partition);

        let flash = FlashType::new(flash, ekv::Config::default());
        if flash.mount().await.is_ok() {
            info!("Flash mounted successfully");
        } else {
            warn!("Failed to mount flash. Assuming first boot, formatting...");
            flash.format().await.expect("Failed to format flash");
        }

        Self { flash }
    }

    pub fn eject(self) -> FlashType<'static, ESPFlashType> {
        self.flash
    }
}
