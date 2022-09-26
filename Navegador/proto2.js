var usuario = 'ESP32Proto';
contrasena = '12345678';

    // called when the client connects
    function onConnect() {
    // Once a connection has been made, make a subscription and send a message.
    console.log("onConnect");
    client.subscribe("#");
    }
    
    // called when the client loses its connection
    function onConnectionLost(responseObject) {
    if (responseObject.errorCode !== 0) {
        console.log("onConnectionLost:", responseObject.errorMessage);
        setTimeout(function() { client.connect() }, 5000);
    }
    }
    
    // called when a message arrives
    function onMessageArrived(message) {
      switch (usuario) {
        case 'ESP32Proto':
          if (message.destinationName == '/' + usuario + '/' + 'LectdB') { //acá coloco el topic
            document.getElementById("LectdB").textContent = message.payloadString  + " [dB]";     
            newData( message.payloadString, 1);}
            break;
        case 'ESP32Proto2':
          if (message.destinationName == '/' + usuario + '/' + 'LectdB'){
                document.getElementById("LectdB").textContent = message.payloadString  + " [dB]";
                newData( message.payloadString, 2);}
            break;
        
            default:
                break;
        }
    }
    

    function onFailure(invocationContext, errorCode, errorMessage) {
        var errDiv = document.getElementById("error");
        errDiv.textContent = "Could not connect to WebSocket server, most likely you're behind a firewall that doesn't allow outgoing connections to port 39627";
        errDiv.style.display = "block";
    }
    
    var clientId = "ws" + Math.random();
    // Create a client instance
    var client = new Paho.Client("driver.cloudmqtt.com", 38813, clientId);
    
    // set callback handlers
    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;
    
    // connect the client
    client.connect({
        useSSL: true,
        userName: usuario,
        password: contrasena,
        onSuccess: onConnect,
        onFailure: onFailure
    });        

//#region Data Service 
var LectdB1Max = 0;
var LectdB2Max = 0;

function newData(data, protoNumber) {
    try {
        numberLecture = Number.parseFloat(data);
        evalMaxValue(numberLecture, protoNumber);
        pushData(numberLecture, protoNumber);
    } catch (error) {
        console.error('Data is not a number');   
    }    
}

function evalMaxValue (newLecture, protoNumber) {    
    switch (protoNumber) {
        case 1:
            if(LectdB1Max < newLecture) {
                LectdB1Max = newLecture;
                document.getElementById("LectdB1Max").textContent = LectdB1Max  + " [dB]";     
            }   
            break;
        case 2:
            if(LectdB2Max < newLecture) {
                LectdB2Max = newLecture;
                document.getElementById("LectdB2Max").textContent = LectdB2Max  + " [dB]";     
            }   
            break;
    
        default:
            break;
    }
}
//#endregion

//#region Charts Services
var dataCounting1 = 0;
var dataCounting2 = 0;
const DATA_COUNT = 60;
const labels = ['Proto2'];

for (let i = 0; i < DATA_COUNT; ++i) {
  labels.push(i.toString());
}

var datapoints = [];

const data = {
  labels: labels,
  datasets: [
    {
      label: 'Cubic interpolation',
      data: datapoints,
      borderColor: '#23aaf1',
      fill: false,
      tension: 0.4
    }
  ]
};

const config1 = {
    type: 'line',
    data: data,
    options: {
      responsive: true,
      plugins: {
        title: {
          display: true,
          text: 'Proto1'
        },
      },
      interaction: {
        intersect: false,
      },
      scales: {
        x: {
          display: true,
          title: {
            display: true
          }
        },
        y: {
          display: true,
          title: {
            display: true,
            text: 'Value'
          },
          suggestedMin: 25,
          suggestedMax: 100
        }
      }
    },
  };

  const config2 = {
    type: 'line',
    data: data,
    options: {
      responsive: true,
      plugins: {
        title: {
          display: true,
          text: 'Proto2'
        },
      },
      interaction: {
        intersect: false,
      },
      scales: {
        x: {
          display: true,
          title: {
            display: true
          }
        },
        y: {
          display: true,
          title: {
            display: true,
            text: 'Value'
          },
          suggestedMin: 25,
          suggestedMax: 100
        }
      }
    },
  };

var myChart1 = new Chart("myChart1", config1);
var myChart2 = new Chart("myChart2", config2);

function pushData(newLecture, protoNumber) {
  switch (protoNumber) {
    case 1:
      if (dataCounting1 < DATA_COUNT) {
        dataCounting1 += 1;
      } else {
        myChart1.data.datasets[0].data.shift();
      }
        myChart1.data.datasets[0].data.push(newLecture);
        myChart1.update();    
      break;
      
    case 2:
      if (dataCounting2 < DATA_COUNT) {
        dataCounting2 += 1;
      } else {
        myChart2.data.datasets[0].data.shift();
      }
        myChart2.data.datasets[0].data.push(newLecture);
        myChart2.update();     
      break;
  
    default:
      break;
  }
}
//#endregion