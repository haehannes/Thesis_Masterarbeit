
var doofinder_script = '//cdn.doofinder.com/media/js/doofinder-classic.7.latest.min.js';

(function(d,t){var f=d.createElement(t),s=d.getElementsByTagName(t)[0];f.async=1;
    f.src=('https:'==location.protocol?'https:':'http:')+doofinder_script;
    f.setAttribute('charset','utf-8');
    s.parentNode.insertBefore(f,s)}(document,'script'));

var dfClassicLayers = [{
    "queryInput": 'input.main-search--field',
    "hashid": '45a7ade20c1a227beacbf756d0b54a92',
    "zone": "eu1",
    "urlHash": false,
    "showInMobile": false,

    "display": {
        "lang": 'de',
        "width": "80%",
        "closeOnClick": true,
        "align": "center",
    },

		"searchParams": {
        "rpp": 20		},

    "callbacks": {
        "loaded": function(config){
            // $("#" + config.initial.mainContainerId).click(function(){$(config.initial.queryInput).focus();});
						        }
    }
}];

