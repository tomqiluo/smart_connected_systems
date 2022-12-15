# Radar Display

## `npm init -y`

## `npm i express body-parser nodemon`

## create index.js

`
const express = require('express')
const bodyParser = require('body-parser')
const app = express()
const port = 3000

app.use(bodyParser.json())
app.use(express.static('public'))

app.listen(port, () => {
  console.log(`App listening on port ${port}`)
})
`

## in package.json

`
"scripts": {
    "start": "nodemon index.js"
},
`

## crate /public

put radar_data.csv and radar_display.html in it

## `npm start`

## go to localhost:3000/radar_display.html to access the chart

# Soucre
https://blog.shahednasser.com/how-to-read-and-write-csv-files-using-node-js-and-express/