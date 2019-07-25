using System;
using System.Net;
using KRPC.Client;
using KRPC.Client.Services.KRPC;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using Vector3D; //dotnet add package Vector3D --version 1.0.2
class Program
{    
    static void Main(string[] args)
    {   
        // connetions and basic static definitions(will not change in th flight)
        var connection = new Connection();
        var spaceCenter = connection.SpaceCenter();
        var vessel = spaceCenter.ActiveVessel;
        var krcp = connection.KRPC();
        var rf = vessel.Orbit.Body.ReferenceFrame;
        var fl = vessel.Flight(rf);
        //starting debug messages
        Console.WriteLine("Connection is established.");
        Console.WriteLine("Version: " + krcp.GetStatus().Version);
        Console.WriteLine(fl.Latitude + " " + fl.Longitude);
        //waiting for user to select a target
        while(spaceCenter.TargetVessel == null){
            //do nothing while we have no target
        }
        //correcting speed mode to avoid SAS error
        vessel.Control.SpeedMode = SpeedMode.Surface;
        //targeting target vessel
        var target_vessel = spaceCenter.TargetVessel;
        //activating next stage, if not working run "dotnet add package Google.Protobuf --version 3.8.0"
        vessel.Control.ActivateNextStage();
        // waiting to develope max trust
        System.Threading.Thread.Sleep(1000);
        while(vessel.Thrust != 0){
            //do nothing until no thrust
        }
        //stage
        vessel.Control.ActivateNextStage();
        // waiting until spacecraft is falling toward body
        while (vessel.Flight(rf).VerticalSpeed > -1) {
            //do nothing
        }
        //stage to activate decending engine
        vessel.Control.ActivateNextStage();
        //setting up AutoPilot
        vessel.AutoPilot.Engage();
        vessel.AutoPilot.ReferenceFrame = rf;
        //decend loop
        while(vessel.Flight(rf).SurfaceAltitude > 1){
            //calculating and normalizing velocity
            var Velocity = vessel.Velocity(rf);
            var VecVelocity = new Vector(Velocity.Item1,Velocity.Item2,Velocity.Item3);
            VecVelocity = Vector.VectorNormalize(VecVelocity);
            var ve = new Tuple<double,double,double>(VecVelocity.X*4,VecVelocity.Y*4,VecVelocity.Z*4);
            //calculating and normalizing vector that points toward target
            var target_vector = new Vector(target_vessel.Position(rf).Item1-vessel.Position(rf).Item1
            ,target_vessel.Position(rf).Item2-vessel.Position(rf).Item2
            ,target_vessel.Position(rf).Item3-vessel.Position(rf).Item3);
            target_vector = Vector.VectorNormalize(target_vector);
            var targ = new Tuple<double,double,double>(target_vector.X*4,target_vector.Y*4,target_vector.Z*4);
            //reflecting vector to point autopilot to it
            var reflected = Vector.VectorDiff(Vector.VectorMul(1.3f*Vector.VectorDot(target_vector,VecVelocity),VecVelocity),target_vector);
            reflected = Vector.VectorMul(-1, reflected);
            reflected = Vector.VectorNormalize(reflected);
            var refe = new Tuple<double,double,double>(reflected.X*2,reflected.Y*2,reflected.Z*2);
            //setting autopilot to it
            vessel.AutoPilot.TargetDirection = refe;
            //drawing vectors
            connection.Drawing().Clear();
            connection.Drawing().AddDirection(ve,rf,15);
            connection.Drawing().AddDirection(targ,rf,15);
            connection.Drawing().AddDirection(refe,rf,15);
            //minimal hight calculation
            /*
                Calculating work that is necessary to stop the rocket by equation:
                m*h*g + 0.5*m*v^2
                m*h*g -> normal equation of potential energy
                0.5*m*v^2 -> equation of kinetic energy
             */
            var m = vessel.Mass;
            var impact_speed = 0.5f; //speed of rocket impact
            var h = vessel.Flight(rf).SurfaceAltitude;
            var v = vessel.Flight(rf).Speed-impact_speed;
            var g = vessel.Orbit.Body.SurfaceGravity;
            var energy = m*h*g + 0.5f*m*Math.Pow(v,2);
            //then to calculate hight to engage engines:
            var min_hg = energy/vessel.MaxThrust;
            //if lower than hight -> full throttle
            if(h < min_hg){
                vessel.Control.Throttle = 1f;
                if(v < 0){
                    //making sure that rocket doesn't go up again
                    vessel.Control.Throttle = 0;
                    //break;
                }
                
            }
            else{
                //controll thrust
                vessel.Control.Throttle = 0.07f;
                
            }
            if(h < 1425){
                // making sure that engine doesn't oversteer
                foreach(var engine in vessel.Parts.Engines){
                    engine.GimbalLimit = 0.2f;
                }
                //switching from targeting the target to canceling velocity
                vessel.AutoPilot.Disengage();
                vessel.Control.SAS = true;
                System.Threading.Thread.Sleep(300);
                vessel.Control.SASMode = SASMode.Retrograde;
                
            }
            if(h < 40){
                //pointing upwards for touch-down
                vessel.Control.SASMode = SASMode.Radial;
            }
            //braking if leg touches the ground
            bool do_brake = false;
            foreach(var leg in vessel.Parts.Legs){
                if(leg.IsGrounded){
                    vessel.Control.Throttle = 0;
                    do_brake = true;
                    break;
                }
            }
            if(do_brake){break;}
        }
        //shouting down the engine after touch-down
        vessel.Control.Throttle = 0;
        
        
        
            

        
    }
}

    
