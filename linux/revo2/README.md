# Revo2 Example Programs

```shell
make clean # Clean old build artifacts

make # Build with default mode (Modbus)
make run_revo2_ctrl # Run Modbus control example
make run_revo2_ctrl_multi # Run Modbus multi-hand control example
make run_revo2_dfu # Run Modbus firmware upgrade example
make run_revo2_touch # Run Modbus touch-hand example

make MODE=can # Build with CAN/CANFD interface mode
make run_revo2_can_ctrl # Run CAN control example
make run_revo2_can_dfu # Run CAN firmware upgrade example
make run_revo2_canfd # Run CANFD control example
make run_revo2_canfd_dfu # Run CANFD firmware upgrade example

make MODE=ethercat # Build with EtherCAT interface mode
make run_revo2_ethercat
```
