var buffer = require('buffer');
var dgram = require('dgram');
var readline = require('readline');
var fs = require('fs');

var rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
});

var PORT = 5000;
var ADDRESS = "192.168.1.103"; //Address of Server

var input = '';

let buf = Buffer.from(input);
let client = dgram.createSocket('udp4');

function waitForUserInput() {
    rl.question("Please input the intensity of the LED (integer from 0 to 9): ", function(answer) {
        if (answer == '0' || answer == '1' || answer == '2' || answer == '3' || answer == '4'  || answer == '5'  || answer == '6'  || answer == '7'  || answer == '8'  || answer == '9') {
            console.log('Intersity:',answer);
            input = answer;
            buf = Buffer.from(input);
            client.send(buf, PORT, ADDRESS, (error) => {
                if(error){
                    client.close();
                  }else{
                    console.log('Data sent !!!');
                  }
            });
            setTimeout(function() {
                waitForUserInput();
            }, 100);
        } else {
            console.log('Invalid Input. Try again.');
            waitForUserInput();
        }
    });
}

waitForUserInput();