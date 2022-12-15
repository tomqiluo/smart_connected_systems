var http = require('http');
var fs = require('fs');

http.createServer(function (req,res) {
    res.writeHead(200,{'Content-Type': 'text/plain'});
    res.end('Hello World!');
}).listen(8080);

var csvdata1 = '';
var csvdata2 = '';
var num1 = 0;
var num2 = 0;

const { SerialPort } = require('serialport')
const { ReadlineParser } = require('@serialport/parser-readline')
const port = new SerialPort({
    path: 'COM5',
    baudRate: 115200
});

const parser = port.pipe(new ReadlineParser({delimiter: '\n'}))
parser.on('data', readSerialData)

function readSerialData(data) {
    console.log(data);
    // count for the number of lines in csv file
    num1 = csvdata1.split(/\r\n|\r|\n/).length;
    if (csvdata1 == '') {
        csvdata1 = data;
    } else {
        csvdata1 = csvdata1 + '\n' + data;
    }
    fs.writeFile('radar_data.csv', csvdata1, err => {
        if (err) {
          console.error(err);
        }
      });
    if (num1 >= 6) {
        csvdata1 = '';
    }

    // count for the number of lines in csv file
    num2 = csvdata2.split(/\r\n|\r|\n/).length;
    if (csvdata2 == '') {
        csvdata2 = data;
    } else {
        csvdata2 = csvdata2 + '\n' + data;
    }
    fs.writeFile('temp_data.csv', csvdata2, err => {
        if (err) {
          console.error(err);
        }
      });

    if (num2 >= 20) {
        csvdata2 = '';
    }
}