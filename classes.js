
class robotBase
{
    constructor(x,y,theta)
    {
        this.pose = {x:x,y:y,theta:theta} //x,y arbitrary units, theta in rad
        this.velocity = null;
        this.ICC = {x:Infinity,y:Infinity};
    }
    #calcRotation() {
        console.log("This Calculates Rotation Params for internal use");
    }
    updatePose(deltaTime)
    {
        console.log("This Calculates new pose given deltaT");
    }
    #getRotationMatrix(omega,deltaT)
    {
        console.log("This Calculates Rotation matrix");
    }


}
class Mat
{
    constructor(N,M, Arr=null)
    {
        this.dims = [N,M];
        if(Arr===null)
            this.values = new Array(N*M).fill(0);
        else
            this.values = Arr;
    }
    #getInternalIdx(idx)
    {
        if(idx.length !=2 ||idx[0]>=this.dims[0]||idx[1]>=this.dims[1])
            throw new Error("invalid index")

        let unfoldedIdx = idx[0]*this.dims[1]+idx[1]; 
        return unfoldedIdx;
       /* let internalidx = {idx:0}
        if(idx.length === this.dims.length)
            {
                idx.forEach((elem, i,arr, dims) => {
                    internalidx.idx += elem*dims.slice(i).reduce((a,b)=>a*b);
                },this.dims);

                return internalidx.idx;
            }
        else
        {
            throw new Error("index does not match dims")
        }*/
    }
    getItem(idx)
    {
        let intIdx = this.#getInternalIdx(idx);
        return this.values[intIdx];
    }
    setItem(idx,val)
    {
        let intIdx = this.#getInternalIdx(idx);
        this.values[intIdx] = val
    }
    static Add(mat1,mat2)
    {
        if(mat1.dims.every((ele,i,arr)=> ele==mat2.dims[i]))
            {
                let result = new Mat(mat1.dims[0],mat1.dims[1]);
                result.values = mat1.values.map((ele,i,arr)=>ele+mat2.values[i]);
                return result
            }
        else
        {
            throw new Error("size Mismatch");
        }
    }
    static Mul(mat1,mat2)
    {
        if(typeof(mat2)==='number')
            {
                var temp = mat1;
                mat1 = mat2
                mat2 = temp;
            }
        if(typeof(mat1)==='number' && mat2 instanceof Mat)
            {
                let result = new Mat(mat2.dims[0],mat2.dims[1]);
                mat2.values.forEach((ele,i,arr) => {
                    result.values[i] = ele * mat1;
                });
                return result
            }
        if(mat1.dims[1]!=mat2.dims[0])
            throw new Error("size Mismatch");
        let result = new Mat(mat1.dims[0],mat2.dims[1])

        for (let i = 0; i < mat1.dims[0]; i++) {
            for (let j = 0; j < mat2.dims[1]; j++) {
                let temp = 0;
                for( let k =0;k<mat1.dims[1];k++)
                    {
                        temp+= mat1.getItem([i,k])*mat2.getItem([k,j]);
                    }
            result.setItem([i,j],temp);
            }
        }
        return result;
    }
}
class Vect3 extends  Mat
{
    constructor(Arr=null)
    {
        super(3,1,Arr);
    }
}
class WheeledRobot extends robotBase
{
    constructor(x,y,theta,wheelRadius)
    {
        super(x,y,theta);
        this.wheelRadius = wheelRadius;
    }
}
class DiffDrive extends WheeledRobot
{
    constructor(length,wheelRadius)
    {
        super(0,0,0,wheelRadius);
        this.length = length;
        this.velocity = {left:0,right:0};
    }
    #calcRotation() {
        let omega = (this.velocity.right-this.velocity.left)/this.length;
        let R = (1/2)* (this.velocity.left+this.velocity.right)/omega;     
        
        return {R:R,omega:omega};
    }
    updatePose(deltaTime)
    {
        let x = this.pose.x;
        let y = this.pose.y;
        let theta = this.pose.theta;
        let RotParams = this.#calcRotation();
        
        if(isNaN(RotParams.R) || Math.abs(RotParams.R)===Infinity)
        {
            let disp = deltaTime*this.velocity.left
            this.pose.x = x+disp*Math.cos(theta);
            this.pose.y = y+disp*Math.sin(theta);
            this.ICC = {x:this.pose.x ,y:this.pose.y};
        }
        else{
        
            let ICC = {x:x- RotParams.R*Math.sin(theta),y:y+RotParams.R*Math.cos(theta)} //Instantanious Center of curvature 
            let rotMatrix = this.#getRotationMatrix(RotParams.omega,deltaTime);
            let dispPose = new Vect3([x-ICC.x,y-ICC.y,theta])
            
            let TransformedFrame = Mat.Mul(rotMatrix,dispPose);
            let dispBack = new Vect3([ICC.x, ICC.y,RotParams.omega*deltaTime]);
            
            TransformedFrame = Mat.Add(TransformedFrame,dispBack)
            this.pose.x = TransformedFrame.getItem([0,0]);
            this.pose.y = TransformedFrame.getItem([1,0]);
            this.pose.theta = TransformedFrame.getItem([2,0]);
            this.ICC = ICC;
        }

    }
    #getRotationMatrix(omega,deltaT)
    {
        let RotMatrix = new Mat(3,3);
        let angularDisplacement = omega*deltaT
        let sinComp = Math.sin(angularDisplacement);
        let cosComp = Math.cos(angularDisplacement);

        RotMatrix.setItem([0,0],cosComp)
        RotMatrix.setItem([0,1],-1*sinComp)
        RotMatrix.setItem([1,0],sinComp)
        RotMatrix.setItem([1,1],cosComp)
        RotMatrix.setItem([2,2],1)
        return RotMatrix;
    }
}
class Sensor
{
    constructor(id,localPose){
        this.id = null;
        this.localPose = localPose;
    }
    getReading(framePose,Env)
    {
        console.log("returns sensor reading for current pos in the environ");
    }
}
class Environment
{
    constructor(linePath, callbackfn, thickness=10, segmentSize=4, svgTemplate = `
            <svg height="400" width="450" xmlns="http://www.w3.org/2000/svg">
                <!-- Draw the paths -->
                <path id="lineAB" d="%Path%" stroke="black" stroke-width="%thickness%"/>

            </svg>
        `, interpolatorConfig=
        {
        joinPathData: false,
        minDistance: 1,
        roundToNearest: 1,
        sampleFrequency: 0.0001
    })
    {
        this.svg = svgTemplate.replace("%Path%",linePath).replace("%thickness%",thickness.toString());
        this.paths = null
        this.segmentSize =segmentSize
        import("./svg-interpolator/index.js").then((res) => 
            {
                res.createInterpolator(interpolatorConfig).then( (interpolator) =>
                    {
                        this.paths = interpolator.interpolatePath(linePath);
                        callbackfn(this);
                    }
                )

            });
        
    }
}
class LineSensor extends Sensor
{
    constructor(id,PoseOffset={x:0,y:0,theta:0},size=5){
        super(id,PoseOffset);
        this.size = size;
           
    }
    getReading(framePose,env)
    {
        if(env.paths ===null)
            return;
        var SensorX = framePose.x + this.localPose.x;
        var SensorY = framePose.y + this.localPose.y;
        var theta = framePose.theta + this.localPose.theta;
        var intersection = 0
        const paths = env.paths
        for (let index = 0; index < paths.length-2; index+=2) {
            //implement intersection logic
            var x = paths[index]
            var y = paths[index+1]
            intersection+= this.#calcRectIntersection([SensorX,SensorY,this.size],[x,y,env.segmentSize])
        }
        const percentArea = intersection/(this.size*this.size);
        return Math.min(Math.round(percentArea*100),100);
    }
    #calcRectIntersection(sqr1,sqr2) //rect [x,y,size]
    {
        const [x1,y1,size1] = sqr1
        const [x2,y2,size2] = sqr2
        const x1TL = x1 - (size1/2);
        const y1TL = y1 - (size1/2);

        const x2TL = x2 - (size2/2);
        const y2TL = y2 - (size2/2);

        const x2BR = x2 + (size2/2);
        const y2BR = y2 + (size2/2);

        const x1BR = x1 + (size1/2);
        const y1BR = y1 + (size1/2);

        const xIntersect = Math.max(x1TL, x2TL);
        const yIntersect = Math.max(y1TL, y2TL);

        const xIntersectEnd = Math.min(x1BR, x2BR);
        const yIntersectEnd = Math.min(y1BR, y2BR);

        const intersectWidth = Math.max(0, xIntersectEnd - xIntersect);
        const intersectHeight = Math.max(0, yIntersectEnd - yIntersect);
        return intersectWidth * intersectHeight;
    }
}

class Plant
{
    err = 0;
    segmaErr = 0;
    deltaErr = 0;
    #lastErr = 0;
    constructor(setpoint,segmaClip=100)
    {
        this.setpoint = setpoint;
        this.segmaClip = segmaClip;
        
    }
    updatePlant(newData)
    {
        this.err = newData - this.setpoint;
        this.deltaErr = this.err - this.#lastErr;
        this.segmaErr=Math.max(Math.min(this.segmaErr+this.err,this.segmaClip),-1*this.segmaClip);
        this.#lastErr = this.err;
    }
}

class PidController
{
    constructor(K_p,K_i,K_d,plant)
    {
        this.K_p = K_p;
        this.K_i = K_i;
        this.K_d = K_d;
        this.plant = plant;
    }
    getCorrection()
    {
        return this.K_p*this.plant.err + this.K_i*this.plant.segmaErr + this.K_d*this.plant.deltaErr;
    }
} 