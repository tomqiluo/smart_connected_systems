var http = require('http');
var fs = require('fs');

http.createServer(function (req,res) {
    res.writeHead(200,{'Content-Type': 'text/plain'});
    res.end('Hello World!');
}).listen(8080);

var csvdata = '';
var num = 0;

const { SerialPort } = require('serialport')
const { ReadlineParser } = require('@serialport/parser-readline')
const port = new SerialPort({
    path: 'COM5',
    baudRate: 115200
});

const parser = port.pipe(new ReadlineParser({delimiter: '\n'}))
parser.on('data', readSerialData)

function readSerialData(data) {
    // count for the number of lines in csv file
    num = csvdata.split(/\r\n|\r|\n/).length;
    if (csvdata == '') {
        csvdata = data;
    } else {
        csvdata = csvdata + '\n' + data;
    }
    console.log(num);
    console.log(csvdata);
    fs.writeFile('radar_data.csv', csvdata, err => {
        if (err) {
          console.error(err);
        }
      });
    if (num >= 3) {
        csvdata = '';
    }
}