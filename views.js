class robotView
{
    constructor(initPose={x:app.screen.width / 2,y:app.screen.height / 2})
    {
        this.sprite = null;
        this.model = new DiffDrive(50,1);
        this.sensor = new LineSensor("sensor1",{x:0,y:0,theta:0},20);
        this.ICC = null
        this.ICCrad = null;
        this.sensorView = null
        this.baseV = 0;
        const texturepromise = PIXI.Assets.load('sample.png');
        texturepromise.then((resolved) =>
            {
            this.sprite = PIXI.Sprite.from(resolved);
            this.sprite.anchor.set(0.5);
            this.sprite.x = initPose.x;
            this.sprite.y = initPose.y;
            this.model.pose.x = this.sprite.x;
            this.model.pose.y = this.sprite.y;
            this.model.pose.theta = this.sprite.rotation;
            this.sensorView = new PIXI.Graphics().rect(0,0,this.sensor.size,this.sensor.size).stroke({ width: 1, color: "yellow" });
            this.sensorView.x = this.sprite.x+this.sensor.localPose.x-this.sensor.size/2
            this.sensorView.y = this.sprite.y+this.sensor.localPose.y-this.sensor.size/2
            this.ICC = new PIXI.Graphics().circle(0,0,5).fill("red");
            this.ICC.x = this.sprite.x
            this.ICC.y = this.sprite.y
            this.ICCrad = new PIXI.Graphics();
            app.stage.addChild(this.sprite);
            app.stage.addChild(this.ICC);
            app.stage.addChild(this.ICCrad);
            app.stage.addChild(this.sensorView);
            console.log("Asset loaded!!")
        });
    }
        updatePose(deltaTime)
        {
            if(this.sprite!==null){
                this.model.updatePose(deltaTime);
                this.sprite.x = this.model.pose.x; 
                this.sprite.y = this.model.pose.y;
                this.sprite.rotation = this.model.pose.theta;
                this.ICC.x = this.model.ICC.x;
                this.ICC.y = this.model.ICC.y;
                this.ICCrad.x =0
                this.ICCrad.y =0
                this.ICCrad.clear()
                this.ICCrad.moveTo(this.ICC.x,this.ICC.y).lineTo(this.sprite.x, this.sprite.y).stroke({ width: 4, color: "red" });
            
                this.sensorView.x = this.sprite.x+this.sensor.localPose.x-this.sensor.size/2
                this.sensorView.y = this.sprite.y+this.sensor.localPose.y-this.sensor.size/2
            }  
        }
        getReadings(env)
        {
            return this.sensor.getReading(this.model.pose,env);
        }        
        reset(initPose)
        {
          this.sprite.x = initPose.x;
          this.sprite.y = initPose.y;
          this.model.pose.x = this.sprite.x;
          this.model.pose.y = this.sprite.y;
          this.ICC.x = this.sprite.x
          this.ICC.y = this.sprite.y
        }
}
const arrayRange = (start, stop, step) =>
    Array.from(
    { length: (stop - start) / step + 1 },
    (value, index) => start + index * step
    );

class chartView
{
    constructor(cnvId,bufferSize,label,color='#FF0000')
    {
        this.bufferSize = bufferSize;
        const ctx = document.getElementById(cnvId);
      this.data = {
        labels:arrayRange(0,bufferSize,1),
          datasets: [{
            label: label,
            data: [],
            borderWidth: 1,
            pointBorderColor: color,
            borderColor: color
          }]
        }
      this.chart = new Chart(ctx, {
        type: 'line',
        data:this.data,
        options: {
          scales: {
            y: {
                beginAtZero: true
            }
            
          },
          animation: {
              duration: 100
        }}
      });
    }
    updateChart(newData){
        if(this.data.datasets[0].data[this.data.datasets[0].data.length-1] == newData)
            return;
        if(this.data.datasets[0].data.length>this.bufferSize)
            {
                this.data.datasets[0].data.shift();
            }
            this.data.datasets[0].data.push(newData);
            this.chart.update();
    }
}
const app = new PIXI.Application();
await app.init({
    antialias: true,
    backgroundColor: 'white',
    width: 800, height: 800 
});

class EnvView
{
  constructor(path)
  {
    this.graphics = null
    this.env = new Environment(path,(env)=>
      {
          this.graphics = new PIXI.Graphics().svg(env.svg);
          app.stage.addChild(this.graphics);
          robo = new robotView({x:env.paths[0],y:env.paths[1]});
      }
    )
  }
  reset()
  {
    app.stage.removeChild(this.graphics);
    this.graphics.destroy();
  }
}
const container = document.getElementById("groundContainer");
container.appendChild(app.canvas);
var testPath = localStorage.getItem('lineFollowPath');
if(!testPath)
 testPath = "M 50 300 C 100 225 150 50 250 150 C 300 200 150 450 350 450 C 350 275 400 300 400 100 C 400 50 500 150 550 50 C 550 50 550 550 600 350 A 50 50 0 1 1 650 550";  

var robo = null
var envView = new EnvView(testPath);
var env = envView.env;


var err = new chartView('errorChart',100,'Error','#FF0000');
var segmaErr = new chartView('segmaChart',100,'segmaError','#00FF00');
var deltaErr = new chartView('deltaChart',100,'deltaError',"#0000FF");
var errPlant = new Plant(50);
var controller = new PidController(0,0,0,errPlant);
var V_Base = document.getElementById("V_Base");
var refresh_btn = document.getElementById("refresh_btn");
var path_textarea = document.getElementById("path_textarea");
path_textarea.value = testPath;
V_Base.onchange = function() {
    robo.baseV = Number(this.value);
  }

var Kp_input = document.getElementById("kp");
var Ki_input = document.getElementById("ki");
var Kd_input = document.getElementById("kd");

Kp_input.onchange = function() {
    controller.K_p = this.value;
  }
Ki_input.onchange = function() {
    controller.K_i = this.value;
  }
Kd_input.onchange = function() {
    controller.K_d = this.value;
  }

refresh_btn.onclick = function() {
  robo.reset({x:env.paths[0],y:env.paths[1]});
  }

  path_textarea.oninput = function () {
    localStorage.setItem('lineFollowPath', this.value);
    location.reload();
  }
app.ticker.add((ticker) => {
    if(robo !=null)
        {
            robo.updatePose(ticker.deltaMS/100);
            var newData = robo.getReadings(env);
            errPlant.updatePlant(newData)
            err.updateChart(errPlant.err);
            segmaErr.updateChart(errPlant.segmaErr);
            deltaErr.updateChart(errPlant.deltaErr);
            const correction = controller.getCorrection()
            robo.model.velocity.left = robo.baseV - correction;
            robo.model.velocity.right =robo.baseV + correction;;
        }
});