// A robot on the Clunkyard core's D binding. The core owns tracking,
// geometry and protocol handling; this strategy chooses the tactics.
// It searches with a broad scan, narrows the beam on a contact, and leads
// shots from a line through its recent sightings.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude wall_contact applied hull_mounts
//   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) debug.log(line) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import clunkyard;
import std.math:abs,sqrt,floor,atan2,PI;
import std.algorithm:min,max;
private double get(alias query)(void*robot){double value;check(robot,query(robot,&value));return value;}
private double steer(double towards,double current){double delta,value;check(null,cy_angles_angle_diff(towards,current,&delta));check(null,cy_angles_clamp(delta/45,-1,1,&value));return value;}
private void drive(void*r,double throttle,double turn,double pitch){check(r,cy_drive_throttle(r,throttle));check(r,cy_drive_turn(r,turn));check(r,cy_drive_pitch_turn(r,pitch));}
private void scan(void*r,double heading,double arc,double pitch){check(r,cy_scanner_heading(r,heading));check(r,cy_scanner_arc(r,arc));check(r,cy_scanner_pitch(r,pitch));}
private void show(void*r,string text){check(r,cy_status_text(r,text.ptr,cast(uint)text.length));}
private void show(void*r,string text,double value){show(r,text);check(r,cy_status_value(r,value));}
// Bearing is earned: a scan contact's bearing is off by up to its bearing_error
// (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
// one aims at it. Contacts within 120 units are exact.
enum SEARCH_ARC=40.0;  // degrees: sweeps the circle in half a second
enum HOLD_WIDTH=60.0;  // units across at the target: it stays in the beam from tick to tick
enum MIN_ARC=6.0;      // degrees: the narrowest the beam holds
enum WINDOW=12;        // ticks of sightings the aim averages
enum SIGHTINGS=8;      // narrow sightings in the window before a shot
enum LOOK_EVERY=40;    // ticks between looks around the target, for a nearer robot
enum REACQUIRE=10;     // ticks to look again where a lost target was
// A straight line through the narrowest of the last WINDOW ticks' sightings:
// where the robot is now and how fast it moves. Each sighting is off by its
// bearing_error; the line averages that out.
struct Fit{CyVec3 position,velocity;int count;}
bool fit(void*r,int slot,int tick,ref Fit result){
    CySighting[32] all;uint total;int rate;
    if(cy_tracker_history(r,slot,all.ptr,all.length,&total)!=CY_OK)return false;check(r,cy_match_tick_rate(r,&rate));
    CySighting[] recent;double best=double.infinity;
    foreach(s;all[0..total])if(s.tick>tick-WINDOW){recent~=s;best=min(best,s.bearing_error);}
    if(recent.length<2)return false;
    CySighting[] seen;foreach(s;recent)if(s.bearing_error<=1.5*best)seen~=s;
    if(seen.length<2)return false;
    double t0=0;foreach(s;seen)t0+=s.tick;t0/=seen.length;
    double spread=0;foreach(s;seen)spread+=(s.tick-t0)*(s.tick-t0);
    // Least squares on one axis: its fitted position now and its slope per tick.
    void line(double function(ref const CySighting) axis,out double now,out double slope){
        double mean=0;foreach(s;seen)mean+=axis(s);mean/=seen.length;
        double along=0;foreach(s;seen)along+=(s.tick-t0)*(axis(s)-mean);
        slope=along/spread;now=mean+slope*(tick-t0);
    }
    double sx,sy,sz;
    line((ref const CySighting s)=>s.position.x,result.position.x,sx);
    line((ref const CySighting s)=>s.position.y,result.position.y,sy);
    line((ref const CySighting s)=>s.position.z,result.position.z,sz);
    result.velocity=CyVec3(sx*rate,sy*rate,sz*rate);result.count=cast(int)seen.length;return true;
}
class Starter{
    bool freeFlight;int dodgeTicks,dodges,slot;
    void start(void*r){CyVec3 gravity;int is3d;check(r,cy_arena_gravity(r,&gravity));check(r,cy_match_is_3d(r,&is3d));freeFlight=is3d!=0&&gravity.z==0;}
    void turn(void*r){
        CyVec3 position;CyContact target;CyCue cue;CyScan last;check(r,cy_me_position(r,&position));check(r,cy_scanner_last(r,&last));
        const heading=get!cy_me_heading(r),pitch=get!cy_me_pitch(r);int tick;check(r,cy_clock_tick(r,&tick));check(r,cy_drive_brake(r,0.2));
        int lastSeen=-1000;if(slot!=0&&cy_tracker_last_seen(r,slot,&lastSeen)!=CY_OK)lastSeen=-1000;
        if(cy_sensors_nearest(r,&target)==CY_OK){
            slot=target.slot;
            const throttle=target.range>350?0.7:target.range<200?-0.4:0.2;drive(r,throttle,steer(target.bearing,heading),steer(target.elevation,pitch));
            // Halve the beam each tick until it is HOLD_WIDTH across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            const hold=max(MIN_ARC,2*atan2(HOLD_WIDTH/2,target.range)*180/PI);
            // In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks, widen it for a tick.
            const arc=tick%LOOK_EVERY==0?SEARCH_ARC:max(hold,min(SEARCH_ARC,last.arc/2));scan(r,target.bearing,arc,target.elevation);
            double power;check(r,cy_gun_power_for_flight_time(r,target.range,1,&power));
            // An exact contact: cy_gun_lead is the intercept if the target holds course.
            // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            CySolution aim;bool ready=true,aimed;Fit line;
            if(target.bearing_error>0){aimed=fit(r,target.slot,tick,line)&&cy_gun_lead_point(r,&line.position,&line.velocity,power,&aim)==CY_OK;ready=aimed&&line.count>=SIGHTINGS;}
            else aimed=cy_gun_lead(r,&target,power,&aim)==CY_OK;
            if(!aimed){aim.heading=target.bearing;aim.pitch=target.elevation;}
            check(r,cy_gun_aim_heading(r,aim.heading));check(r,cy_gun_aim_pitch(r,aim.pitch));CyDirection now;check(r,cy_gun_aim_now(r,&now));double delta;check(r,cy_angles_angle_diff(aim.heading,now.heading,&delta));int canFire;check(r,cy_gun_can_fire(r,power,10,&canFire));
            if(ready&&abs(delta)<3&&abs(aim.pitch-now.pitch)<3&&canFire!=0){check(r,cy_gun_fire(r));check(r,cy_gun_power(r,power));}show(r,ready?"TRACK":"NARROW",target.range);
        }else if(slot!=0&&tick-lastSeen<=REACQUIRE){
            // Lost it: look again where it was last seen, widening the beam, before searching.
            CyVec3 where;double bearing,elevation=0;check(r,cy_tracker_position(r,slot,&where));check(r,cy_angles_bearing_to(&position,&where,&bearing));if(freeFlight)check(r,cy_angles_elevation_to(&position,&where,&elevation));
            drive(r,0.5,steer(bearing,heading),steer(elevation,pitch));scan(r,bearing,min(SEARCH_ARC,last.arc*2),elevation);show(r,"REACQUIRE");
        }else if(cy_tracker_cue(r,&cue)==CY_OK){
            double bearing,distance,elevation=0,arc;check(r,cy_angles_bearing_to(&position,&cue.position,&bearing));check(r,cy_angles_distance(&position,&cue.position,&distance));if(freeFlight)check(r,cy_angles_elevation_to(&position,&cue.position,&elevation));
            const error=min(cue.bearing_error,30),across=(tick%5-2)/2.0*error,up=freeFlight?((tick/5)%3-1)*error/2:0;
            drive(r,0.7,steer(bearing,heading),steer(elevation,pitch));check(r,cy_scanner_arc_for_reach(r,distance+100,&arc));scan(r,bearing+across,arc,elevation+up);show(r,"CUE",distance);
        }else{
            double sweep;check(r,cy_angles_normalize(last.heading+36,&sweep));drive(r,0.5,0.33,steer(0,pitch));scan(r,sweep,SEARCH_ARC,freeFlight?((tick/10)%3-1)*35:0);show(r,"SEARCH");
        }
        // A simple dodge: vary it, or predict the shooter.
        CyContact[32] cues;uint count;bool fired;
        if(cy_sensors_cues(r,cues.ptr,cues.length,&count)==CY_OK)foreach(heard;cues[0..count])fired|=heard.source==CY_SOURCE_MUZZLE;
        if(dodgeTicks==0&&fired){dodges++;dodgeTicks=10;}
        if(dodgeTicks>0){
            // Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
            const left=dodges%2==1;const double side=left?1:-1;int control;check(r,cy_match_control(r,&control));dodgeTicks--;
            if(control==CY_CONTROL_FORCES){check(r,cy_drive_throttle(r,0.5));check(r,cy_drive_strafe(r,-side));}
            else{check(r,cy_drive_throttle(r,1));check(r,cy_drive_turn(r,side));}
            show(r,left?"dodge:left":"dodge:right",dodges);
        }
        if(get!cy_battery_charge(r)<get!cy_battery_capacity(r)/4)check(r,cy_drive_throttle(r,get!cy_drive_get_throttle(r)*0.5));
        CyStorm storm;double margin;if(cy_nav_storm(r,&storm)==CY_OK&&cy_nav_storm_margin(r,&position,&margin)==CY_OK&&margin>-60){double away,delta,control;check(r,cy_angles_bearing_to(&storm.center,&position,&away));check(r,cy_angles_angle_diff(away,heading,&delta));check(r,cy_angles_clamp(delta/30,-1,1,&control));check(r,cy_drive_throttle(r,0.8));check(r,cy_drive_turn(r,control));}
        CyZone zone;if(cy_nav_zone(r,&zone)==CY_OK&&cy_nav_zone_margin(r,&position,&margin)==CY_OK&&margin<60){double bearing,climb=0,delta,turn,pitchTurn;check(r,cy_angles_bearing_to(&position,&zone.center,&bearing));if(zone.shape==CY_SHAPE_SPHERE)check(r,cy_angles_elevation_to(&position,&zone.center,&climb));check(r,cy_angles_angle_diff(bearing,heading,&delta));check(r,cy_angles_clamp(delta/30,-1,1,&turn));check(r,cy_angles_clamp((climb-pitch)/30,-1,1,&pitchTurn));drive(r,0.8,turn,pitchTurn);}
    }
}
void main(){auto robot=new Starter;run(&robot.turn,&robot.start);}
