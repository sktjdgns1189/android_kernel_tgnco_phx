rm ./zImage
rm ./dt.img
./dtbTool -2 -o ./dt.img -s 2048 -p ./scripts/dtc/ ./arch/arm/boot/
cp ./arch/arm/boot/zImage ./
