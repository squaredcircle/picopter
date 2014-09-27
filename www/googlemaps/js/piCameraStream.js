jwplayer.key="d/Dh7oqvADc7HHCzewmdNZS8LypdBt0+gj9Zng==";

jwplayer('video-jwplayer').setup({
	flashplayer:"/jwplayer/jwplayer.flash.swf"
	, file:"rtmp://" + window.location.hostname + "/flvplayback/flv:myStream.flv"
	, autoStart: true
	, rtmp:{
		bufferlength:0.1
	}
	, deliveryType: "streaming"
	, width: "100%"//960
	, height: "100%"//540
	, player: {
		modes: {
			linear: {
				controls:{
					stream:{
						manage:false
						, enabled: false
						}
					}
				}
			}
		}
	, shows: {
		streamTimer: {
			enabled: true
			, tickRate: 100
		}
	}
});
