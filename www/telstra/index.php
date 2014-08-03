<!DOCTYPE html>
<html>
	<head>
		<!-- Header. CSS includes. -->
		<meta charset="utf-8">
		<title>UWA LCARS</title>
		<link rel="stylesheet" type="text/css" href="css/bootstrap.min.css">
		<link rel="stylesheet" type="text/css" href="css/bootstrap-theme.min.css">
		<link rel="stylesheet" type="text/css" href="css/style.css">
	</head>
	<body>
		<!-- Body -->
		<div id="panel" style="margin-left: -52px">
			<button id="drop" onclick="addMarker()">Drop Marker</button>
		</div>
		<div id="map-canvas"></div>
		<div id="controls">
			<div id="controlpanel">
				
			</div>
			<div id="videofeed">
				<video autoplay="true" id="videoElement"></video>
			</div>
		</div>
		
		
		<!-- Footer. Javascript includes. -->
		<script src="js/userwebcam.js"></script>
		<script src="js/jquery-2.1.1.min.js"></script>
		<script src="js/bootstrap.min.js"></script>
		<script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false"></script>
		<script src="js/map.js"></script>
	</body>
</html>