# [doc = "Register `SYNCBUSY` reader"] pub type R = crate :: R < SyncbusySpec > ; # [doc = "Field `SWRST` reader - Software Reset Synchronization Busy"] pub type SwrstR = crate :: BitReader ; # [doc = "Field `ENABLE` reader - SERCOM Enable Synchronization Busy"] pub type EnableR = crate :: BitReader ; impl R { # [doc = "Bit 0 - Software Reset Synchronization Busy"] # [inline (always)] pub fn swrst (& self) -> SwrstR { SwrstR :: new ((self . bits & 1) != 0) } # [doc = "Bit 1 - SERCOM Enable Synchronization Busy"] # [inline (always)] pub fn enable (& self) -> EnableR { EnableR :: new (((self . bits >> 1) & 1) != 0) } } # [doc = "I2CS Syncbusy\n\nYou can [`read`](crate::Reg::read) this register and get [`syncbusy::R`](R). See [API](https://docs.rs/svd2rust/#read--modify--write-api)."] pub struct SyncbusySpec ; impl crate :: RegisterSpec for SyncbusySpec { type Ux = u32 ; } # [doc = "`read()` method returns [`syncbusy::R`](R) reader structure"] impl crate :: Readable for SyncbusySpec { } # [doc = "`reset()` method sets SYNCBUSY to value 0"] impl crate :: Resettable for SyncbusySpec { const RESET_VALUE : u32 = 0 ; }