var uwa = new google.maps.LatLng(-31.978239, 115.817546);
var markers = [];
var map;

function initialize() {
	var mapOptions = {
		zoom: 20,
		center: uwa
	};

	map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);
	addMarker();
}

function addMarker() {
	markers.push(
		new google.maps.Marker( {
			position: uwa,
			map: map,
			draggable: true,
			animation: google.maps.Animation.DROP
		} )
	);
}

google.maps.event.addDomListener(window, 'load', initialize);
