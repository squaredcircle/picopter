<!DOCTYPE html>
<html>
	<head>
		<!-- Header. CSS includes. -->
		<meta charset="utf-8">
		<title>Copter LCARS</title>
		<meta name="viewport" content="width=device-width, initial-scale=1.0">
		<link rel="stylesheet" type="text/css" href="css/bootstrap.min.css">
		<link rel="stylesheet" type="text/css" href="css/bootstrap-theme.min.css">
		<link rel="stylesheet" type="text/css" href="css/style.css">
		<link rel="stylesheet" type="text/css" href="css/leaflet.css">
	</head>
	<body>
		<!-- Body -->
		<div id="mainwindow">
			<div id="panel">
				<button id="drop" onclick="addMarker()">Drop Marker</button>
			</div>
			<div id="map"></div>
		</div>
		<div id="sidepanel">
			<div id="sidepanel_top">
				<div id="gps_coords" style="font-family: Courier; font-size:15px;">
				</div>
			</div>
			<div id="sidepanel_bottom">
				<object type="application/x-shockwave-flash" data="jwplayer/jwplayer.flash.swf" width="100%" height="100%" bgcolor="#000000" id="video-jwplayer" name="video-jwplayer" tabindex="0">
					<param name="allowfullscreen" value="true">
					<param name="allowscriptaccess" value="always">
					<param name="seamlesstabbing" value="true">
					<param name="wmode" value="opaque">
				</object>
				<div id="video-jwplayer_aspect" style="display: none;"></div>
				<div id="video-jwplayer_jwpsrv" style="position: absolute; top: 0px; z-index: 10;"></div>
			</div>
		</div>
		
		
		<!-- Footer. Javascript includes. -->
		<script src="jwplayer/jwplayer.js"></script>
		<script src="js/piCameraStream.js"></script>
		<script src="js/jquery-2.1.1.min.js"></script>
		<script src="js/bootstrap.min.js"></script>
		<script src="js/map.js"></script>
		<script src="js/leaflet.js"></script>
		<script>

		var map = L.map('map').setView([-31.978239, 115.817546], 13);

		L.tileLayer('https://{s}.tiles.mapbox.com/v3/{id}/{z}/{x}/{y}.png', {
			maxZoom: 18,
			id: 'examples.map-i86knfo3'
		}).addTo(map);
		
		L.marker([-31.978239, 115.817546]).addTo(map)
			.bindPopup("<b>Hello world!</b><br />I am a popup.").openPopup();

		L.circle([-31.978239, 115.817546], 100, {
			color: 'red',
			fillColor: '#f03',
			fillOpacity: 0.5
		}).addTo(map).bindPopup("I am a circle.");

		var popup = L.popup();

		function onMapClick(e) {
			popup
				.setLatLng(e.latlng)
				.setContent("You clicked the map at " + e.latlng.toString())
				.openOn(map);
		}

		map.on('click', onMapClick);

	</script>
		
		<script>
			$(document).ready(function(){
				$('.button').click(function(){
					var clickBtnValue = $(this).val();
					var ajaxurl = 'ajax.php',
					data = {
						'action': clickBtnValue,
					};
					$.post(ajaxurl, data, function (response) {
						 $('#response').append("<p><b>[AJAX execution]</b> Response received: <i>" + response + "</i>");
					});
				});
			});
			
			(function worker() {
				$.ajax({
					url:'ajax-thrift.php',
					success: function(data) {
						$('#gps_coords').html(data);
					},
					complete: function() {
						$.ajax({
							url:'ajax_map.php',
							success: function(data) {
								var arr = data.split(',');
								var newLatLng = new google.maps.LatLng(parseFloat(arr[0]),parseFloat(arr[1]));
								markers[0].setPosition(newLatLng);
							}
						});
						setTimeout(worker, 1000);
					}
				});
			})();
		</script>
	</body>
</html>
