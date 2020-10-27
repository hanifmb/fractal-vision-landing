
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});




function startMission() {

  alert('hello!'); 

// Calling a service
  // -----------------

  var addTwoIntsClient = new ROSLIB.Service({
    ros : ros,
    name : '/start_vision_landing',
    serviceType : 'std_srvs/Trigger'
  });

  var request = new ROSLIB.ServiceRequest({});

  addTwoIntsClient.callService(request, function(result) {
      console.log(result.message);
  });

  
}


window.onload = function(){

  document.getElementById("startMission").onclick = startMission;

};

