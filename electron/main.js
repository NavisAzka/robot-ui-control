const { app, BrowserWindow } = require('electron')
const path = require('path')

function createWindow() {
  const win = new BrowserWindow({
    width: 1400,
    height: 900,
  })

  win.loadURL('http://localhost:5173')
}

app.whenReady().then(createWindow)
