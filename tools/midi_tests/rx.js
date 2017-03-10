var midi = require('midi'),
    async = require('async'),
    inquirer = require('inquirer'),
    progress = require('multi-progress')(process.stdout),
    output = new midi.output(),
    speed = 100,
    portNames = [];

for(var i=0; i < output.getPortCount(); i++) {
  portNames.push(output.getPortName(i));
}

if(! portNames.length) {
  console.error('no MIDI ports available');
  process.exit(1);
}

inquirer.prompt([
  {
    type: 'input',
    name: 'speed',
    message: 'Send interval (ms)',
    default: speed
  },
  {
    type: 'list',
    name: 'port',
    message: 'Select port',
    choices: portNames
  }
]).then(answers => {
  output.openPort(portNames.indexOf(answers.port));
  speed = parseInt(answers.speed);
  startTest();
});

function startTest() {

  var on_bar = progress.newBar('note on  [:bar] :current/:total', {
    incomplete: ' ',
    total: 128
  });

  var off_bar = progress.newBar('note off [:bar] :current/:total', {
    incomplete: ' ',
    total: 128
  });

  async.timesSeries(128, function(n, next){

    setTimeout(function() {
      on_bar.tick();
      output.sendMessage([0x90, n, 0x7F]);
      next();
    }, speed);

  }, function() {});

  async.timesSeries(128, function(n, next){

    setTimeout(function() {
      off_bar.tick();
      output.sendMessage([0x90, n, 0x00]);
      next();
    }, speed);

  }, function() {});

}
