// if enabled or ip are not defined, set them
if (localStorage.getItem('enabled') == null || localStorage.getItem('ip') == null) {
    localStorage.setItem('enabled', 'true')
    localStorage.setItem('ip', 'localhost')
}

const filters = {
    urls: ['https://translate-service.scratch.mit.edu/translate?language=*&text=*']
}

chrome.webRequest.onBeforeRequest.addListener(function (details) {
    // is the extension currently enabled?
    var enabled = (localStorage.getItem('enabled') == 'true')
    console.log('enabled', enabled)

    // extract the text paramater from the URL
    var url = details.url
    var text = (new URLSearchParams(url)).get("text")
    var language = (new URLSearchParams(url)).get("language")

    if (enabled && (text.startsWith('[') || text.startsWith('{') || text.startsWith('%'))) {
        // proxy this request to a url of our choosing
        var ip = localStorage.getItem('ip')
        text = encodeURIComponent(text)
        language = encodeURIComponent(language)
        return {
            redirectUrl: 'http://' + ip + '/translate?language=' + language + '&text=' + text
        }
    }
        // do nothing (ie. continue the request) if [ isn't the first char
}, filters, ['blocking'])
