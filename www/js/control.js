var canEdit = false;

function manualMode() {
	$("#manual-holder").show();
	$("#auto-holder").hide();
	$("#status-holder").hide();
	showMarkers();
}

function autoMode() {
	$("#manual-holder").hide();
	$("#auto-holder").show();
	$("#status-holder").hide();
	hideMarkers();
}

function statusMode() {
	$("#manual-holder").hide();
	$("#auto-holder").hide();
	$("#status-holder").show();
}

function toggleEdit() {
	$("#manual-edit").toggleClass('btn-pressed');
	$("#auto-edit").toggleClass('btn-pressed');
	
	$("#main-vertical").toggleClass('orange-toggle');
	$("#main-side").toggleClass('orange-toggle');
	$("#main-bottom").toggleClass('orange-toggle');
	
	$("button[id*='begin']").toggleClass('btn-disabled');
	
	canEdit = !canEdit;
	
	toggleMarkerRed();
	
	if (canEdit) {
		$("#information").html('<span id="blinkbox" style="background-color: #FB8500;">&nbsp;&nbsp;&nbsp;&nbsp;</span> Edit mode engaged. Use the map.');
		$("#blinkbox").blink({delay:1000});
		
		$.each( markers, function( index, value ){
			value.dragging.enable();
		});
	} else {
		$("#blinkbox").unblink();
		$("#information").html('');
		
		$.each( markers, function( index, value ){
			value.dragging.disable();
		});
		
		ajaxSend('updateWaypoints', markers);
	}
}