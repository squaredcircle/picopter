var uwa = [-31.979839, 115.817546];
var markers = [];
var bounds = [];
var copterMarker;
var map = L.map('map-canvas').setView(uwa, 18);

var boundRect;

var popup = L.popup();

/* ************************************* INITIALISATION */

function initialise() {
	L.tileLayer('/tiles/sat/gs_{x}_{y}_{z}.jpg', {
		maxZoom: 21,
		minZoom: 17
	}).addTo(map);
	
	addCopter();
}

$(function() {
    initialise();
});

/* ************************************* NEW MARKERS */

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

function addMarker(data,loc,red) {
	if (red) {
		data.push(
			new L.marker( loc, {
				draggable: true,
				icon:	new L.NumberedDivIconRed({number: (data.length+1)})
			} ).addTo(map)
		);
	} else {
		data.push(
			new L.marker( loc, {
				draggable: false,
				icon:	new L.NumberedDivIcon({number: (data.length+1)})
			} ).addTo(map)
		);
	}

	var thisNo = data.length-1;	
	var thisMarker = data[thisNo];

	thisMarker.on('click', function(event) {
		if (canEdit) {
			map.removeLayer(thisMarker);
			data.splice( $.inArray(thisMarker,data) ,1 );
			hideRectangle();
		}
	});
	
	thisMarker.on('drag', function(e) {
		if (canEditBounds) updateRectangle();
	});
}

/* ************************************* MODIFICATIONS */

function toggleMarkerRed(data) {
	tmpmarkers = data.slice();
	clearMarkers(data,false);
	$.each( tmpmarkers, function( index, value ){
		addMarker(data,value.getLatLng(), canEdit);
	});
}

function clearMarkers(data, ajax) {
	for (var i = 0; i < data.length; i++) {
		map.removeLayer(data[i]);
	}
	
	while(data.length > 0) {
		data.pop();
	}
	if (ajax) ajaxSend('resetWaypoints');
}

/* ************************************* VISIBILITY */

function hideMarkers(data) {
	for (var i = 0; i < data.length; i++) {
		map.removeLayer(data[i]);
	}
}

function showMarkers(data) {
	for (var i = 0; i < data.length; i++) {
		data[i].addTo(map);
	}
}

function updateRectangle() {
	if (bounds.length == 2) {
		map.removeLayer(boundRect);
		boundRect = L.rectangle(L.latLngBounds(bounds[0].getLatLng(), bounds[1].getLatLng()), {color: "#0066FF", weight: 1}).addTo(map);
	}
}

function hideRectangle() {
	map.removeLayer(boundRect);
}

function showRectangle() {
	if (bounds.length == 2) boundRect = L.rectangle(L.latLngBounds(bounds[0].getLatLng(), bounds[1].getLatLng()), {color: "#0066FF", weight: 1}).addTo(map);
}

/* ************************************* CLICKETY-CLICK */

function onMapClick(e) {
	if (canEditMarkers) {
		addMarker(markers, e.latlng, true);
	} else if (canEditBounds) {
		if (bounds.length == 0) {
			addMarker(bounds, e.latlng, true);
		
		} else if (bounds.length == 1) {
			addMarker(bounds, e.latlng, true);
			showRectangle();
		
		} else {
			updateRectangle();
		}
	}
}

map.on('click', onMapClick);

