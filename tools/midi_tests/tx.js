var midi = require('midi'),
    async = require('async'),
    inquirer = require('inquirer'),
    progress = require('multi-progress'),
    input = new midi.input(),
    multi = new progress(),
    portNames = [];

for(var i=0; i < input.getPortCount(); i++) {
  portNames.push(input.getPortName(i));
}

inquirer.prompt([
  {
    type: 'list',
    name: 'port',
    message: 'Select port',
    choices: portNames
  }
]).then((answers) => {
  input.openPort(portNames.indexOf(answers.port));
  startTest();
});

function startTest() {

  var on_bar = multi.newBar('midi on  [:bar] :current/:total', {
    incomplete: ' ',
    total: 128
  });

  var off_bar = multi.newBar('midi off [:bar] :current/:total', {
    incomplete: ' ',
    total: 128,
    callback: function() {
      input.closePort();
      process.exit();
    }
  });

  input.on('message', function(deltaTime, message) {
    if(message[0] != 144) return;
    if(message[2] == 0) off_bar.tick();
    if(message[2] == 100) on_bar.tick();
  });

}
