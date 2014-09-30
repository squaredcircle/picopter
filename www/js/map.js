var uwa = [-31.979839, 115.817546];
var markers = [];
var copterMarker;
var map = L.map('map-canvas').setView(uwa, 18);

var popup = L.popup();

function initialise() {
	L.tileLayer('/tiles/sat/gs_{x}_{y}_{z}.jpg', {
		maxZoom: 21,
		minZoom: 17
	}).addTo(map);
	
	addCopter();
}

function onMapClick(e) {
	if (canEdit) addMarker(e.latlng, true);
}

map.on('click', onMapClick);

function addCopter() {
	var copterIcon = L.icon({
		iconUrl:	'/css/helicopter.png',
		iconSize:	[32,37],
		iconAnchor:	[16,35]
	});
	
	copterMarker =
		new L.marker( uwa, {
			draggable: false,
			icon: copterIcon
		}).addTo(map);
}

function addMarker(loc,red) {
	//map.getCenter()
	if (red) {
		markers.push(
			new L.marker( loc, {
				draggable: true,
				icon:	new L.NumberedDivIconRed({number: (markers.length+1)})
			} ).addTo(map)
		);
	} else {
		markers.push(
			new L.marker( loc, {
				draggable: true,
				icon:	new L.NumberedDivIcon({number: (markers.length+1)})
			} ).addTo(map)
		);
	}

	var thisNo = markers.length-1;	
	var thisMarker = markers[thisNo];
/*
	if (thisNo > 1) {
		var waypointLine = new google.maps.Polyline({
			path: [markers[thisNo-1],markers[thisNo]],
			geodesic: true,
			strokeColor: '#FF0000',
			strokeOpacity: 1.0,
			strokeWeight: 2
		});

		waypointLine.setMap(map);
	}*/
	
	//ajaxSend('addWaypoint',thisMarker.getLatLng().lat,thisMarker.getLatLng().lng);

	thisMarker.on('click', function(event) {
		if (canEdit) {
			map.removeLayer(thisMarker);
			markers.splice( $.inArray(thisMarker,markers) ,1 );
		}
	});
	
	//thisMarker.on('dragend', function(event) {
	//	ajaxSend('updateWaypoint',thisMarker.getLatLng().lat,thisMarker.getLatLng().lng,thisNo);
	//});
}

function toggleMarkerRed() {
	tmpmarkers = markers.slice();
	clearMarkers(false);
	$.each( tmpmarkers, function( index, value ){
		addMarker(value.getLatLng(), canEdit);
	});
}

// Removes the markers from the map, but keeps them in the array.
function clearMarkers(ajax) {
	for (var i = 0; i < markers.length; i++) {
		map.removeLayer(markers[i]);
	}
	
	while(markers.length > 0) {
		markers.pop();
	}
	if (ajax) ajaxSend('resetWaypoints');
}

$(function() {
    initialise();
});
