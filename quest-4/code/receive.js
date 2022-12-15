var buffer = require('buffer');
var dgram = require('dgram');
var fs = require('fs');
var cors = require('cors');
var express = require('express');

var app = express();

app.use(express.json());

var PORT = 5000;
var ADDRESS = "192.168.1.102";      //Address of ESP32

var result = '';

let buf = Buffer.from(result);
let client = dgram.createSocket('udp4');

app.use(cors());
app.options('*', cors());

app.get('/', function(request, response){
    var input = request.get("value");
    // console.log(request.get("value"));
    response.sendStatus(200);
    // response.send(request.body);
    if (input == "Stop") {
        var result = "0";
    } else if (input == "Start") {
        var result = "1";
    } else if (input == "Left") {
        var result = "2";
    } else if (input == "Right") {
        var result = "3";
    }
    buf = Buffer.from(result);
    client.send(buf, PORT, ADDRESS, (error) => {
        if(error){
            client.close();
        }else{
            console.log(input+': '+result+'       Data sent !!!');
        }
    });
});

app.listen(8080);