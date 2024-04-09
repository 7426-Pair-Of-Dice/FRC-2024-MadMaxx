
let paths = [];
let autos = [];
const directory = '../../src/main/deploy/pathplanner/'
const autopath = directory + 'autos/'
const pathpath = directory + 'paths/'
const fs = require('fs');

function getFiles(folder) {
    let files = []
    fs.readdirSync(folder).forEach(file => {
        files.push(file)
    });
    return files
}

function writeJSON(file, object) {
    fs.writeFile(file, JSON.stringify(object), 'utf8', ()=>{});
}

function duplicateAuto(autonomous, append) {
  const data = fs.readFileSync(autopath + autonomous, 'utf8');
  let auto = JSON.parse(data)
  auto.command.data.commands.forEach((command)=>{
    if(command.type === "path") {
        duplicatePath(command.data.pathName + '.path', append)
        command.data.pathName = command.data.pathName + append
    }
  })
  writeJSON(autopath + autonomous.split(".")[0] + append + '.auto', auto)
}

function duplicatePath(path, append) {
    const data = fs.readFileSync(pathpath + path, 'utf8');
    let oldpath = JSON.parse(data)
    oldpath.waypoints.forEach((waypoint)=>{
        if(waypoint.linkedName !== null) {
            waypoint.linkedName = waypoint.linkedName + append
        }
    })
    writeJSON(pathpath + path.split(".")[0] + append + '.path', oldpath)
}


autos = getFiles(autopath)
paths = getFiles(pathpath)

const prompt = require('readline').createInterface({
    input: process.stdin,
    output: process.stdout
  });
  

autos.forEach((auto, index)=>{
    console.log('['+index+'] '+auto)
})

prompt.question('Which Auto would you like to duplicate?\nIndex: ', autoIndex => {
    duplicateAuto(autos[autoIndex], " - RED")
    prompt.close();
});