const filters = {
    urls: [
        "*://translate-service.scratch.mit.edu/translate*"
    ]
}

chrome.webRequest.onBeforeRequest.addListener(function (details) {
    var url = details.url
    var text = (new URLSearchParams(url)).get("text")

    try {
        console.log(text);
        if (text.substring(0,1) == "[") {
            // proxy this request to a url of our choosing
            return {
                redirectUrl: "https://localhost/translate?language=fr&text=" + text
            }
        }
    } catch (e) {
        // do nothing (ie. continue the request) if no [ as 1st char in text
    }
}, filters, ["blocking"])


chrome.webRequest.onHeadersReceived.addListener(function(details) {
    console.log(1)
    // add the header
    details.responseHeaders.push({
        name: 'Access-Control-Allow-Origin',
        value:'*'
    })
    console.log(details.responseHeaders)

    // return the new headers
    return {
        responseHeaders: details.responseHeaders
    }
}, {urls: ['*://127.0.0.1/*', '*://localhost/*']},['responseHeaders', 'blocking'])