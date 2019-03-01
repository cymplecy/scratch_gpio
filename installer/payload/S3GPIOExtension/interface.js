// if enabled or ip are not defined, set them
if (localStorage.getItem('enabled') == null || localStorage.getItem('ip') == null) {
    localStorage.setItem('enabled', 'true')
    localStorage.setItem('ip', 'localhost')
}

// every time we open the interface window, set the enabled and IP inputs
document.getElementById('enable-box').checked = (localStorage.getItem('enabled') == 'true')
document.getElementById('host-ip').value = localStorage.getItem('ip')

// when reset-ip is clicked, set back to 127.0.0.1
document.getElementById('reset-ip').addEventListener('click', function () {
    document.getElementById('host-ip').value = 'localhost'
    localStorage.setItem('ip', 'locahost')
})

// when IP field is changed, update that in localstorage
document.getElementById('host-ip').addEventListener('change', function () {
    localStorage.setItem('ip', document.getElementById('host-ip').value)
})


document.getElementById('fergusondavid6-link').addEventListener('click', function () {
    var url = 'https://github.com/davidferguson'
    chrome.tabs.create({url})
})

// when you modify check box, reflect that in localstorage
document.getElementById('enable-box').addEventListener('click', function () {
    localStorage.setItem('enabled', document.getElementById('enable-box').checked)
})
