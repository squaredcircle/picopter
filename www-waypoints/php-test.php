<?php
	namespace picopter\php;

	error_reporting(E_ALL);

	require_once '/home/pi/thrift-0.9.0/lib/php/lib/Thrift/ClassLoader/ThriftClassLoader.php';

	use Thrift\ClassLoader\ThriftClassLoader;

	$GEN_DIR = '/home/pi/git/picopter/www-waypoints/gen-php';
	
	$loader = new ThriftClassLoader();
	$loader->registerNamespace('Thrift', '/home/pi/thrift-0.9.0/lib/php/lib');
	$loader->registerDefinition('picopter', $GEN_DIR);
	$loader->register();

	use Thrift\Protocol\TBinaryProtocol;
	use Thrift\Transport\TSocket;
	use Thrift\Transport\THttpClient;
	use Thrift\Transport\TBufferedTransport;
	use Thrift\Exception\TException;

	try {
		$socket = new TSocket('localhost', 9090);	
		$transport = new TBufferedTransport($socket, 1024, 1024);
		$protocol = new TBinaryProtocol($transport);
		$client = new \picopter\webInterfaceClient($protocol);
		
		$transport->open();
		
		/* ***************************************** */
		
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

		/* ***************************************** */
		
		$transport->close();
		
	} catch (TException $tx) {
		print 'TException: '.$tx->getMessage()."\n";
	}
?>
