// Global variables
var map;



// ROS communication
//------------------------------
var ros;
var nUavs = 3;
var selectedUav = 0;
var uavs = [];
var gpsSubscribers = [];
var utmSubscribers = [];
var localPositionSubscribers = [];
var stateSubscribers = [];
var setpointRawPublishers = [];
var setModeClients = [];
var armingClients = [];
var takeoffClients = [];

var v1Subscribers = [];
var v2Subscribers = [];
var v3Subscribers = [];
var v4Subscribers = [];
var vResSubscribers = [];


// Connecting to ROS
ros = new ROSLIB.Ros({
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


// Fill uav, subscribers, publishers and service clients arrays
for ( i = 0; i < nUavs; i++ ) {
	
	// UAV object
	var uav = {
		armed: false,
		mode: "",
		latitude: 0.0,
		longitude: 0.0,
		altitude: 0.0,
		x: 0.0,
		y: 0.0,
		roll: 0.0,
		pitch: 0.0,
		yaw: 0.0,
		utmX: 0.0,
		utmY: 0.0,
		v1: { x: 0.0, y: 0.0, z: 0.0 },
		v2: { x: 0.0, y: 0.0, z: 0.0 },
		v3: { x: 0.0, y: 0.0, z: 0.0 },
		v4: { x: 0.0, y: 0.0, z: 0.0 },
		vRes: { x: 0.0, y: 0.0, z: 0.0 }
	};
	
	// GPS subscriber
	var gpsSubscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/mavros/global_position/global',
		messageType : 'sensor_msgs/NavSatFix'
	});
	gpsSubscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].latitude = message.latitude;
		uavs[idx].longitude = message.longitude;
	});
	
	
	// UTM subscriber
	var utmSubscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/mavros/global_position/local',
		messageType : 'nav_msgs/Odometry'
	});
	utmSubscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].utmX = message.pose.pose.position.x;
		uavs[idx].utmY = message.pose.pose.position.y;
	});
	
	// Local position subscriber
	var localPositionSubscriber = new ROSLIB.Topic({
		ros: ros,
		name: '/uav' + i + '/mavros/local_position/odom',
		messageType: 'nav_msgs/Odometry'
	});
	localPositionSubscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		
		var quaternionpose = new THREE.Quaternion(
			message.pose.pose.orientation.x, 
			message.pose.pose.orientation.y, 
			message.pose.pose.orientation.z, 
			message.pose.pose.orientation.w);
		var eulerpose = new THREE.Euler;
		eulerpose.setFromQuaternion(quaternionpose);
      
		uavs[idx].x = message.pose.pose.position.x;
		uavs[idx].y = message.pose.pose.position.y;
		uavs[idx].altitude = message.pose.pose.position.z;
		uavs[idx].roll = eulerpose.x;
		uavs[idx].pitch = eulerpose.y;
		uavs[idx].yaw = eulerpose.z;
	});
	
	// State subscriber
	var stateSubscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/mavros/state',
		messageType : 'mavros_msgs/State'
	});
	stateSubscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].armed = message.armed;
		uavs[idx].mode = message.mode;
	});
	
	
	// Setpoint Raw Publisher
	var setpointRawPublisher = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/mavros/setpoint_raw/global',
		messageType : 'mavros_msgs/GlobalPositionTarget'
	});
	
	
	//-------------------------------------------------------------
	// Vectors subscribers
	var v1Subscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/v1',
		messageType : 'geometry_msgs/Point32'
	});
	v1Subscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].v1 = { 'x': message.x, 'y': message.y, 'z': message.z };
	});
	
	var v2Subscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/v2',
		messageType : 'geometry_msgs/Point32'
	});
	v2Subscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].v2 = { 'x': message.x, 'y': message.y, 'z': message.z };
	});
	
	var v3Subscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/v3',
		messageType : 'geometry_msgs/Point32'
	});
	v3Subscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].v3 = { 'x': message.x, 'y': message.y, 'z': message.z };
	});
	
	var v4Subscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/v4',
		messageType : 'geometry_msgs/Point32'
	});
	v4Subscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].v4 = { 'x': message.x, 'y': message.y, 'z': message.z };
	});
	
	var vResSubscriber = new ROSLIB.Topic({
		ros : ros,
		name : '/uav' + i + '/vRes',
		messageType : 'geometry_msgs/Point32'
	});
	vResSubscriber.subscribe(function(message) {
		var idx = this.name.substring(4, 5);
		uavs[idx].vRes = { 'x': message.x, 'y': message.y, 'z': message.z };
	});
	
	//------------------------------------------------------------------------
	
	
	
	
	
	// set_mode service client
	var setModeClient = new ROSLIB.Service({
		ros : ros,
		name : '/uav' + i + '/mavros/set_mode',
		serviceType : 'mavros_msgs/SetMode'
	});
	
	// arming service client
	var armingClient = new ROSLIB.Service({
		ros : ros,
		name : '/uav' + i + '/mavros/cmd/arming',
		serviceType : 'mavros_msgs/CommandBool'
	});
	
	// takeoff service client
	var takeoffClient = new ROSLIB.Service({
		ros : ros,
		name : '/uav' + i + '/mavros/cmd/takeoff',
		serviceType : 'mavros_msgs/CommandTOL'
	});
	
	// Fill corresponding arrays
	uavs.push(uav);
	gpsSubscribers.push(gpsSubscriber);
	utmSubscribers.push(utmSubscriber);
	localPositionSubscribers.push(localPositionSubscriber);
	stateSubscribers.push(stateSubscriber);
	setpointRawPublishers.push(setpointRawPublisher);
	setModeClients.push(setModeClient);
	armingClients.push(armingClient);
	takeoffClients.push(takeoffClient);
	
	v1Subscribers.push(v1Subscriber);
	v2Subscribers.push(v2Subscriber);
	v3Subscribers.push(v3Subscriber);
	v4Subscribers.push(v4Subscriber);
	vResSubscribers.push(vResSubscriber);
}
		

function initMap() {
	
	// Initialize Google Map and objects used by it
	map = new google.maps.Map(document.getElementById('map'), {
		center: {lat: -22.414257, lng: -45.446744},
		zoom: 19
	});

	var icon = {
		url: 'images/quad.png',
		scaledSize: new google.maps.Size(30, 30)
	};
	
	
	// Create one marker for each UAV
	var markers = [];
	for (i = 0; i < nUavs; i++) {
		markers.push(new google.maps.Marker({
			icon: icon
		}));
	}
	
	
	// Draw markers on the map 4 times per second
	window.setInterval(function() {
		for ( i = 0; i < nUavs; i++ ) {
			markers[i].setMap(null);
			markers[i].setPosition({lat: uavs[i].latitude, lng: uavs[i].longitude});
			markers[i].setMap(map);
		}
	}, 250);
	
	
	// Right-click context menu
	var contextMenu = google.maps.event.addListener(
        map,
        "rightclick",
        function( event ) {
            // use JS Dom methods to create the menu
            // use event.pixel.x and event.pixel.y 
            // to position menu at mouse position
            menuX = event.pixel.x;
            menuY = event.pixel.y;
            lat = event.latLng.lat();
            lon = event.latLng.lng();
            
            var message = new ROSLIB.Message({
				coordinate_frame: 0,
				type_mask: 0,
				latitude: lat,
				longitude: lon,
				altitude: 0.0,
				velocity: {x: 0.0, y: 0.0, z: 0.0},
				acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0},
				yaw: 0.0,
				yaw_rate: 0.0
			});
			setpointRawPublishers[selectedUav].publish(message);
        }
    );							
}


$(document).ready(function(){
	
	// Fill UAV select element and register click event on each one
	for ( i = 0; i < nUavs; i++ ) {
		$("#uav-select").append('<option data-id="' + i + '">UAV ' + (i + 1) + '</option>');
	}
	$("#uav-select").change(function() {
		selectedUav = $(this).children(":selected").data("id");
	});
	
	
	// Update selected UAV data on screen 4 times per second
	window.setInterval(function() {
		
		//console.log(uavs);
			
			if( uavs[selectedUav].armed ) {
				$("#btn-arming").html("Disarm");
				$("#st-arming").html('<span class="text-success">Armed</span>');
				$("#btn-takeoff").prop("disabled", false).removeClass("btn-disabled");
			} else {
				$("#btn-arming").html("Arm");
				$("#st-arming").html('<span class="text-danger">Disarmed</span>');
				$("#btn-takeoff").prop("disabled", true).addClass("btn-disabled");
			}
			$("#st-mode").html(uavs[selectedUav].mode);
		
			$("#st-latitude").empty().append(uavs[selectedUav].latitude);
			$("#st-longitude").empty().append(uavs[selectedUav].longitude);
			$("#st-altitude").empty().append(uavs[selectedUav].altitude.toFixed(2));
			$("#st-x").empty().append(uavs[selectedUav].x.toFixed(2));
			$("#st-y").empty().append(uavs[selectedUav].y.toFixed(2));
			$("#st-yaw").empty().append( uavs[selectedUav].yaw.toFixed(2) );
			drawAttitudeIndicator( uavs[selectedUav].roll, uavs[selectedUav].pitch );
			
			$("#st-utm-x").empty().append(uavs[selectedUav].utmX.toFixed(2));
			$("#st-utm-y").empty().append(uavs[selectedUav].utmY.toFixed(2));
			
			for ( i = 1; i < uavs.length; i++ ) {
				drawVectorViewer( uavs[selectedUav].v1, 
								  uavs[selectedUav].v2, 
								  uavs[selectedUav].v3, 
								  uavs[selectedUav].v4, 
								  uavs[selectedUav].vRes, i );		
			}
			
			$("#st-v1").empty().append("(" + uavs[selectedUav].v1.x.toFixed(3) + ", " + uavs[selectedUav].v1.y.toFixed(3) + ")"); 
			$("#st-v2").empty().append("(" + uavs[selectedUav].v2.x.toFixed(3) + ", " + uavs[selectedUav].v2.y.toFixed(3) + ")"); 
			$("#st-v3").empty().append("(" + uavs[selectedUav].v3.x.toFixed(3) + ", " + uavs[selectedUav].v3.y.toFixed(3) + ")"); 
			$("#st-v4").empty().append("(" + uavs[selectedUav].v4.x.toFixed(3) + ", " + uavs[selectedUav].v4.y.toFixed(3) + ")"); 
			$("#st-vres").empty().append("(" + uavs[selectedUav].vRes.x.toFixed(3) + ", " + uavs[selectedUav].vRes.y.toFixed(3) + ")"); 
		}, 250);
	
	
	// Set Mode Function
	$("#btn-set-mode").click(function() {
		var request = new ROSLIB.ServiceRequest({
			base_mode : 0,
			custom_mode : $("#mode-select").children(":selected").data("mode")
		});

		setModeClients[selectedUav].callService(request, function(result) {
			console.log('Result for service call on '
			  + setModeClients[selectedUav].name
			  + ': '
			  + result.success);
		});
	});
	
	
	// Arming Function
	$("#btn-arming").click(function() {
		
		// If the UAV is armed, disarm it. If it is not armed, arm it.
		if ( uavs[selectedUav].armed ) {
			var request = new ROSLIB.ServiceRequest({
				value : false
			});
		} else {
			var request = new ROSLIB.ServiceRequest({
				value : true
			});
		}
		
		armingClients[selectedUav].callService(request, function(result) {
			console.log('Result for service call on '
			  + armingClients[selectedUav].name
			  + ': '
			  + result.success);
		});
	});
	
	
	// Take off function
	$("#btn-takeoff").click(function() {
		
		// Only works if UAV is armed
		if ( uavs[selectedUav].armed ) {
			var request = new ROSLIB.ServiceRequest({
				altitude: $('input[name="takeoff-altitude"]').val(),
				latitdue: 0,
				longitude: 0,
				min_pitch: 0,
				yaw: 0
			});
			
			takeoffClients[selectedUav].callService(request, function(result) {
				console.log('Result for service call on '
				  + takeoffClients[selectedUav].name
				  + ': '
				  + result.success);
			});
		} 
	});
	
	// Create the mask that will make the attitude indicator round
	var mask = document.getElementById("mask");
	var ctxm = mask.getContext("2d");
	var wm = mask.width;
	var hm = mask.height;

	// First create a white rectangle
	ctxm.fillStyle = "#fff";
	ctxm.fillRect(0, 0, wm, hm);

	// Then the border of a circle inside it
	ctxm.beginPath();
	ctxm.lineWidth = 2;
	ctxm.arc( wm/2, hm/2, wm/2 - 1, 0, Math.PI*2 );
	ctxm.stroke();
	ctxm.closePath();

	// Now draw a transparent circle inside
	ctxm.globalCompositeOperation = "destination-out";  
	ctxm.fillStyle = "#fff";                           
	ctxm.beginPath();
	ctxm.arc( wm/2, hm/2, wm/2 - 1, 0, Math.PI*2 );
	ctxm.closePath();
	ctxm.fill();
	ctxm.globalCompositeOperation='source-over';

	// Drawn an empty attitude indicator
	drawAttitudeIndicator( 0, 0 );
	
	// Function to draw the attitude indicator
	function drawAttitudeIndicator( roll, pitch ) {
		
		// Get the canvas element
		var canvas = document.getElementById("attitude-indicator");
		var ctx = canvas.getContext("2d");
		w = canvas.width;
		h = canvas.height;
			
		// Clear previously drawn images
		ctx.clearRect(0, 0, w, h);

		// Calculate the line equation
		var a, b, fator;
		a = Math.tan( roll );
		fator = 3 * h / Math.PI;
		b = ( h - a * w ) / 2 + fator * pitch;

		// Calculate the points where the line cross the rectangle
		var x1, y1, x2, y2;
		x1 = 0;
		y1 = b;
		x2 = w;
		y2 = a * w + b;

		// Draw 2 polygons and fill them with the color
		var skyColor = "#669ed1";
		var groundColor = "#4f3822";
		var boxColor = "#5a5a59";
		
		// Draw the sky
		ctx.fillStyle = skyColor;
		ctx.beginPath();
		ctx.moveTo( 0, 0 );
		ctx.lineTo( w, 0 );
		ctx.lineTo( x2, y2 );
		ctx.lineTo( x1, y1 );
		ctx.closePath();
		ctx.fill();
		
		// Draw the ground
		ctx.fillStyle = groundColor;
		ctx.beginPath();
		ctx.moveTo( x1, y1 );
		ctx.lineTo( x2, y2 );
		ctx.lineTo( w, h );
		ctx.lineTo( 0, h );
		ctx.closePath();
		ctx.fill();

		// Draw a box to display the angles in text
		var hb = h/6;
		var wb = w/2;
		var xb = w/4;
		var yb = 3*h/4;
		
		// Draw the box
		ctx.fillStyle = boxColor;
		ctx.beginPath();
		ctx.fillRect( xb, yb, wb, hb );
		ctx.closePath();
		ctx.beginPath();
		ctx.fillStyle="#000000";
		ctx.strokeRect( xb, yb, wb, hb );
		ctx.closePath();
		
		// Write the angles
		roll *= 180 / Math.PI;
		pitch *= 180 / Math.PI;
		ctx.beginPath();
		ctx.font = "bold " + (hb/2 - 1) + "px Arial";
		ctx.textBaseline = "bottom";
		ctx.fillText( "Roll: " + roll.toFixed(1), xb + 5, yb + hb/2 );
		ctx.fillText( "Pitch: " + pitch.toFixed(1), xb + 5, yb + hb );
		ctx.closePath();
		
		// Draw the mask over the attitude indicator
		ctx.drawImage(mask, 0, 0);
	}
	
	// Function to draw the attitude indicator
	function drawVectorViewer( v1, v2, v3, v4, vRes, id ) {
		
		// Get the canvas element
		var canvas = document.getElementById("vector-viewer-" + id);
		var ctx = canvas.getContext("2d");
		w = canvas.width;
		h = canvas.height;
		scale = 10;
			
		// Clear previously drawn images
		ctx.clearRect(0, 0, w, h);

		// Draw 1st vector
		ctx.beginPath();
		ctx.strokeStyle = "#D50032";
		ctx.lineWidth = 2;
		canvas_arrow( ctx, w/2, h/2, w/2 + scale*v1.x, h/2 - scale*v1.y );
		ctx.stroke();
		ctx.closePath();
		
		// Draw 2nd vector
		ctx.beginPath();
		ctx.strokeStyle = "#28B1CA";
		canvas_arrow( ctx, w/2, h/2, w/2 + scale*v2.x, h/2 - scale*v2.y );
		ctx.stroke();
		ctx.closePath();
		
		// Draw 3rd vector
		ctx.beginPath();
		ctx.strokeStyle = "#B14EB5";
		canvas_arrow( ctx, w/2, h/2, w/2 + scale*v3.x, h/2 - scale*v3.y );
		ctx.stroke();
		ctx.closePath();
		
		// Draw 4th vector
		ctx.beginPath();
		ctx.strokeStyle = "#FEE500";
		canvas_arrow( ctx, w/2, h/2, w/2 + scale*v4.x, h/2 - scale*v4.y );
		ctx.stroke();
		ctx.closePath();
		
		// Draw 5th vector
		ctx.beginPath();
		ctx.strokeStyle = "#92B558";
		ctx.lineWidth = 3;
		canvas_arrow( ctx, w/2, h/2, w/2 + scale*vRes.x, h/2 - scale*vRes.y );
		ctx.stroke();
		ctx.closePath();
	}

	function canvas_arrow(context, fromx, fromy, tox, toy){
		var headlen = 0;   // length of head in pixels
		
		var d = Math.sqrt( (toy-fromy) * (toy-fromy) + (tox-fromx) * (tox-fromx) );
		
		if ( d > 10.0 ) {
			headlen = 7;
		}
		
		var angle = Math.atan2(toy-fromy,tox-fromx);
		context.moveTo(fromx, fromy);
		context.lineTo(tox, toy);
		context.moveTo(tox, toy);
		context.lineTo(tox-headlen*Math.cos(angle-Math.PI/6),toy-headlen*Math.sin(angle-Math.PI/6));
		context.moveTo(tox, toy);
		context.moveTo(tox, toy);
		context.lineTo(tox-headlen*Math.cos(angle+Math.PI/6),toy-headlen*Math.sin(angle+Math.PI/6));
	}
});
