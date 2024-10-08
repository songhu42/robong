var fs = require('fs');
var ejs = require('ejs');
var http = require('http');
var mysql = require('mysql');

var express = require('express');
var path = require('path');
var favicon = require('serve-favicon');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');

var routes = require('./routes/index');
var users = require('./routes/users');

var app = express();  

// connect database
var mySqlClient = mysql.createConnection({
	user:'sipt',
	password:'sipt12',
	database:'siptdb'
});

// generate & excute express web server
var app = express();
http.createServer(app).listen(8000,function(){
	console.log('Server running at http://127.0.0.1:8000');
});

// http://localhost:8000/ ---- list.html
app.get('/list',function(req,res)
{
	fs.readFile('public/list.html','utf8',function(error,data)
	{
		if(error)
		{
			console.log('readFile Error');
		}
		else
		{
			mySqlClient.query('select * from dev01', function(error,results)
			{
				if(error)
				{
					console.log('error: ', error.message);
				}
				else
				{
					console.log('results : ', results ); 
					res.send(ejs.render(data,{ dht:results }));
				}
			});
		}
	})
});

// http://locakhost:8000/insert ------ insert.html 
app.get('/insert',function(req,res)
{
	fs.readFile('public/insert.html','utf8',function(error,data)
	{
		if(error)
		{
			console.log('read file error');
		}
		else
		{
			res.send(data);
		}
	})
});

// http://locakhost:8000/edit/3 ---- edit.html
app.get('/edit/:id',function(req,res)
{
	fs.readFile('public/edit.html','utf8',function(error,data)
	{
		mySqlClient.query('select * from dev01 where id = ?',
		[req.params.id], function(error,result)
		{
			if(error)
			{
				console.log('read file error');
			}
			else
			{
				res.send(ejs.render(data,{ dev01:result[0] }));
			}
		});
	});
});

// http://localhost:8000/delete/3 
app.get('/delete/:id',function(req,res)
{
	mySqlClient.query('delete from dev01 where id = ?',
	[req.params.id],function(error,result)
	{
		if(error)
		{
			console.log('delete error');
		}
		else
		{
			console.log('delete id = %d',req.params.id);
			res.redirect('/list');	// return main
		}
	});
});

// handle parameter transfered by POST method
app.use(bodyParser.urlencoded({ extended: true }));

app.post( '/insert', function(req, res)
{
	var body = req.body;
	
	mySqlClient.query( 'insert into dev01(Temperature, Humidity) values(?,?)',
	[body.Temperature, body.Humidity],function(error,result)
	{
		if(error)
		{
			console.log('insert error: ', error.message);
		}
		else
		{
			res.redirect('/list');
		}
	});
});

app.post( '/edit/:id', function(req, res)
{
	var body = req.body;
	
	mySqlClient.query( 'update dev01 set Temperature=?, Humidity=? where id=?',
	[body.Temperature,body.Humidity,body.id],function(error,result)
	{
		if(error)
		{
			console.log('update error: ', error.message);
		}
		else
		{
			res.redirect('/list');
		}
	});
});
		
// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'ejs');

// uncomment after placing your favicon in /public
app.use(favicon(path.join(__dirname, 'public', 'favicon.ico')));
app.use(logger('dev'));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', routes);
app.use('/users', users);

// catch 404 and forward to error handler
app.use(function(req, res, next) {
  var err = new Error('Not Found');
  err.status = 404;
  next(err);
});

// error handlers

// development error handler
// will print stacktrace
if (app.get('env') === 'development') {
  app.use(function(err, req, res, next) {
    res.status(err.status || 500);
    res.render('error', {
      message: err.message,
      error: err
    });
  });
}

// production error handler
// no stacktraces leaked to user
app.use(function(err, req, res, next) {
  res.status(err.status || 500);
  res.render('error', {
    message: err.message,
    error: {}
  });
});


module.exports = app;
