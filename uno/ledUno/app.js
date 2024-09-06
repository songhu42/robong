const express  = require('express');
const app      = express();
var http = require('http').Server(app); 
var io = require('socket.io')(http); 

const path = require('path');

var SerialPort = require('serialport').SerialPort;
var ReadlineParser = require('@serialport/parser-readline').ReadlineParser; 

var parsers    = SerialPort.parsers;

const sp = new SerialPort( { 
  path:'/dev/ttyUSB0',
  baudRate: 115200
});

const port = 3000;

const parser = sp.pipe(new ReadlineParser({ delimiter: '\r\n'})); 

var ledStatus = ""; 
sp.on('open', () => {
	console.log('Port open'); 
});

//소켓 연결시
io.on('connection', (socket) => {
	console.log('a user connected');
	socket.on('disconnect', () => {
		console.log('user disconnected');
	});

	socket.emit('result', `${socket.id}로 연결 되었습니다.`);

	parser.on('data', function(data) {
			var rcv = data.toString(); 
			console.log(rcv);  

			if( rcv.length > 3 ) {
				if( rcv.substring(0, 3) == "led" ) {
					if( rcv.substring(3,4) == "1" ) ledStatus = "ON";
					else ledStatus = "OFF"; 

					console.log("led status : " + ledStatus); 

					io.emit('led', ledStatus); 

				} else if (rcv.substring(0, 3) == "adc") {
					var adc = parseInt(rcv.substring(3)); 
					console.log("adc : " + adc); 
					io.emit('adc', adc); 

				} else if (rcv.substring(0, 3) == "ser") {
					var ser = parseInt(rcv.substring(3)); 
					console.log("ser : " + ser); 
					io.emit('ser', ser); 
				}  
					
			}

	});

	socket.on('message', (msg) => { 
		console.log("클라이언트의 요청이 있습니다.");
		console.log(msg);
		if(msg === 'record'){
			RecordTF = 1;
		}
		else if(msg === 'stop'){
			RecordTF = 0;
		}
		socket.emit('result', `수신된 메세지는 "${ msg }" 입니다.`);
	});
});


app.get('/set_servo/:deg',function(req,res)
{	
	var deg = req.params.deg; 

	console.log("set_servo : " + deg); 

	sp.write(deg + '\n\r', function(err){
		if (err) {
            return console.log('Error on write: ', err.message);
        }

        console.log('send: degree ' + deg );

		res.writeHead(200, {'Content-Type': 'text/plain'});
		res.end('send: degree ' + deg + '\n');

	});
});


app.get('/inc_servo',function(req,res)
{
	sp.write('.\n\r', function(err){
		if (err) {
            return console.log('Error on write: ', err.message);
        }

        console.log('send: increase');

		res.writeHead(200, {'Content-Type': 'text/plain'});
		res.end('Incresed\n');

	});
});

app.get('/dec_servo',function(req,res)
{
	sp.write(',\n\r', function(err){
		if (err) {
            return console.log('Error on write: ', err.message);
        }

        console.log('send: decrese');

		res.writeHead(200, {'Content-Type': 'text/plain'});
		res.end('Decresed\n');

	});
});

app.get('/led_on',function(req,res)
{
	sp.write('o\n\r', function(err){
		if (err) {
            return console.log('Error on write: ', err.message);
        }

        console.log('send: led on');

		res.writeHead(200, {'Content-Type': 'text/plain'});
		res.end('LED ON\n');

	});
});

app.get('/led_off',function(req,res)
{
	sp.write('x\n\r', function(err){
		if (err) {
            return console.log('Error on write: ', err.message);
        }

        console.log('send: led off');

		res.writeHead(200, {'Content-Type': 'text/plain'});
		res.end('LED OFF\n');

	}); 

});

app.use(express.static(__dirname + '/public'));

http.listen(port, function() {
    console.log('listening on *:' + port);
});