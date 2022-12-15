// import modules
var dgram = require('dgram');
var server = dgram.createSocket('udp4');
var fs = require('fs');

// define variables
var PORT = 6000;
var ADDRESS = "192.168.1.114";  // Address of localhost

// Start the server
server.on('listening', function() {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ':' + address.port);
});

// Receive message
server.on('message', function(message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
    fs.appendFile('data.csv', message + '\n', err => {
        if (err) {
          console.error(err);
        }
    });
});

server.bind(PORT, ADDRESS);

// waitForUserInput();