const express = require('express')
const bodyParser = require('body-parser')
const app = express()
const port = 3000

app.use(bodyParser.json())
app.use(express.static('public'))

app.listen(port, () => {
  console.log(`App listening on port ${port}`)
})