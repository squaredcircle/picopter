<?php
	$b = Array(false => 'false', true => 'true');
		
	$ans = $client->allStop();
	print "allStop " . $b[$ans] . "\n";
	
	$ans = $client->beginWaypointTraversal();
	print "beginWaypointTraversal " . $b[$ans] . "\n";
	
	$ans = $client->requestStatus();
	print "resetWaypoints " . $ans . "\n";
	
	$wp = new \picopter\coordDeg();
	$wp->lat = 23423.2352;
	$wp->lon = 654.12;
	$ans = $client->addWaypoint($wp);
	print "addWaypoint " . $b[$ans] . "\n";
	
	$ans = $client->requestCoords();
	print "requestCoords " . $ans->lat . ", " . $ans->lon . "\n";
	
	
	$ans = $client->removeWaypoints();
	print "removeWaypoints " . $b[$ans] . "\n";
	
	$ans = $client->resetWaypoints();
	print "resetWaypoints " . $b[$ans] . "\n";
		
?>