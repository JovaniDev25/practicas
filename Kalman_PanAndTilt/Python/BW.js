/*const { Browser } = require("st-ethernet-ip");

const browser = new Browser();

//When new device is detected
browser.on("New Device", device => {
    //Display all device info
    console.log(device);
    //Display Device IP address
    console.log(device.socketAddress.sin_addr);
    //Display Device Description
    console.log(device.productName)
});

//when device is not detected after x amount of scans
browser.on("Device Disconnected", device => {
    // 'device' is the disconnected device
}) */
/* const { Controller } = require("ethernet-ip");

const PLC = new Controller();

// Controller.connect(IP_ADDR[, SLOT])
// NOTE: SLOT = 0 (default) - 0 if CompactLogix 1769-L23E/B-QB1 LOGIX5323E-QB1
//PLC.connect("192.168.100.101", 0).then(() => {
PLC.connect("1769-L23E/B-QB1 LOGIX5323E-QB1", 0).then(() => {
    console.log(PLC.properties);
}); */

const { Browser } = require("st-ethernet-ip");
const browser = new Browser();

//When new device is detected
browser.on("New Device", device => {
    //Display all device info
    console.log(device);
    //Display Device IP address
    console.log(device.socketAddress.sin_addr);
    //Display Device Description
    console.log(device.productName)
});

//when device is not detected after x amount of scans
browser.on("Device Disconnected", device => {
    // 'device' is the disconnected device
})
