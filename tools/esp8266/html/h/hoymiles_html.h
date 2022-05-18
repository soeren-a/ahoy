#ifndef __HOYMILES_HTML_H__
#define __HOYMILES_HTML_H__
const char hoymiles_html[] PROGMEM = "<!doctype html><html><head><title>Index - {DEVICE}</title><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"/><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><script type=\"text/javascript\">getAjax('/livedata', 'livedata');window.setInterval(\"getAjax('/livedata', 'livedata')\", 10000);function getAjax(url, resid) {var http = null;http = new XMLHttpRequest();if(http != null) {http.open(\"GET\", url, true);http.onreadystatechange = print;http.send(null);}function print() {if(http.readyState == 4) {document.getElementById(resid).innerHTML = http.responseText;}}}</script><style type=\"text/css\"></style></head><body><h1>AHOY - {DEVICE}</h1><div id=\"content\" class=\"content\"><div id=\"livedata\"></div><p>Every 10 seconds the values are updated</p></div><div id=\"footer\"><p class=\"left\">&copy 2022</p><p class=\"left\"><a href=\"/\">Home</a></p><p class=\"right\">AHOY :: {VERSION}</p></div></body></html>";
#endif /*__HOYMILES_HTML_H__*/
