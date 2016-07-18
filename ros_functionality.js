/* Javascript written on 4/17/2016 to connect to the ROS Server 
   Also has functionality that saves data points into an array
   Can display these points on a 3-D plane. */




//Connecting to ROS

var ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });
  ros.on('connection', function() { console.log('Connected to websocket server.'); });
  ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); });
  ros.on('close', function() { console.log('Connection to websocket server closed.'); });


//Listener 
var serverListener = new ROSLIB.Topic({
      ros : ros,
      name : 'producedData',
      messageType : 'std_msgs/UInt32MultiArray' 
});

var messageString = '';
var datapoints = [];

serverListener.subscribe(function(message) {
  console.log('Received message on' + serverListener.name + ': ' + message.data);

  messageString = messageString + "<br>" + message.data;
  datapoints[datapoints.length] = message.data;
    });


//start of the 3D plot generation     
makeChart(7000, 7000, 7000, 'container', datapoints);
makeChart(7000, 7000, 7000, 'container1', datapoints);
   
    //Publishing a Topic
    /* example
     var cmdVel = new ROSLIB.Topic({
  33     ros : ros,
  34     name : '/cmd_vel',
  35     messageType : 'geometry_msgs/Twist'
  36   });
  37 
  38   var twist = new ROSLIB.Message({
  39     linear : {
  40       x : 0.1,
  41       y : 0.2,
  42       z : 0.3
  43     },
  44     angular : {
  45       x : -0.1,
  46       y : -0.2,
  47       z : -0.3
  48     }
  49   });
  50   cmdVel.publish(twist);
   
  //how to make objects in javascript
   var Jaeho = {
    occupation: "student",
    interest: "robotics",
    lastName: "Bang",
    school: "CMU",
    fullDescription: function() {
      return this.occupation + this.interest + this.lastName + this.school;
    }

  //how to create animation in Javascript
<script>
function myMove() {
  var elem = document.getElementById("animate");   
  var pos = 0;
  var id = setInterval(frame, 5);
  function frame() {
    if (pos == 350) {
      clearInterval(id);
    } else {
      pos++; 
      elem.style.top = pos + 'px'; 
      elem.style.left = pos + 'px'; 
    }
  }
}
</script>






};

document.getElementById("demo").innerHTML = Jaeho.fullDescription(); 

    */








