var net = require('net');
var data = '';
var server = net.createServer();  
server.on('connection', handleConnection);

server.listen(8000, function() {  
  console.log('server listening to %j', server.address());
});

function handleConnection(conn) {  
  var remoteAddress = conn.remoteAddress + ':' + conn.remotePort;
  console.log('new client connection from %s', remoteAddress);
  conn.on('data', onConnData);
  conn.once('close', onConnClose);
  conn.on('error', onConnError);
console.log(data.indexOf(0xFF));
console.log(data.indexOf(0xD8));
console.log(data.indexOf(0xD9));

  function onConnData(d) {
//	 if(temp == 0xFF)
//	 {
//		 if(d == 0xD8){
//			console.log('JPEG start'); 
//		 }
//		 else if(d == 0xD9){
//			 console.log('JPEG end');
//		 }
//	 }
//	 temp = d;

	data += d;

    console.log('connection data from %s: %j', remoteAddress, d);
  
  }

  function onConnClose() {
    console.log('connection from %s closed', remoteAddress);
  }

  function onConnError(err) {
    console.log('Connection %s error: %s', remoteAddress, err.message);
  }
}