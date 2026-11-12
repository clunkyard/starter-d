// A robot on the Clunkyard core's D binding. The core owns tracking,
// geometry and protocol handling; this strategy chooses the tactics.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import clunkyard;
import std.math:abs,sqrt,floor;
import std.algorithm:min;
private double get(alias query)(void*robot){double value;check(robot,query(robot,&value));return value;}
private double steer(double towards,double current){double delta,value;check(null,cy_angles_angle_diff(towards,current,&delta));check(null,cy_angles_clamp(delta/45,-1,1,&value));return value;}
private void drive(void*r,double throttle,double turn,double pitch){check(r,cy_drive_throttle(r,throttle));check(r,cy_drive_turn(r,turn));check(r,cy_drive_pitch_turn(r,pitch));}
private void scan(void*r,double heading,double arc,double pitch){check(r,cy_scanner_heading(r,heading));check(r,cy_scanner_arc(r,arc));check(r,cy_scanner_pitch(r,pitch));}
private void show(void*r,string text){check(r,cy_status_text(r,text.ptr,cast(uint)text.length));}
private void show(void*r,string text,double value){show(r,text);check(r,cy_status_value(r,value));}
// The arena's calibration kernel, xorshift-sqrt v1; safe to delete.
private double calibrationWork(ulong work,ulong tick){ulong x=88172645463325252UL^tick;double sum=0;for(ulong i=0;i<work;i++){x^=x<<13;x^=x>>7;x^=x<<17;sum+=sqrt(cast(double)(x>>11));}return cast(double)((cast(ulong)floor(sum)+(x>>40))%1000000);}
class Starter{
    bool freeFlight;
    void start(void*r){CyVec3 gravity;int is3d;check(r,cy_arena_gravity(r,&gravity));check(r,cy_match_is_3d(r,&is3d));freeFlight=is3d!=0&&gravity.z==0;}
    void turn(void*r){
        CyVec3 position;CyContact target;CyCue cue;CyScan last;check(r,cy_me_position(r,&position));check(r,cy_scanner_last(r,&last));
        const heading=get!cy_me_heading(r),pitch=get!cy_me_pitch(r);int tick;check(r,cy_clock_tick(r,&tick));check(r,cy_drive_brake(r,0.2));
        if(cy_sensors_nearest(r,&target)==CY_OK){
            const throttle=target.range>350?0.7:target.range<200?-0.4:0.2;drive(r,throttle,steer(target.bearing,heading),steer(target.elevation,pitch));scan(r,target.bearing,20,target.elevation);
            double power;check(r,cy_gun_power_for_flight_time(r,target.range,1,&power));CySolution aim;if(cy_gun_lead(r,&target,power,&aim)!=CY_OK){aim.heading=target.bearing;aim.pitch=target.elevation;}
            check(r,cy_gun_aim_heading(r,aim.heading));check(r,cy_gun_aim_pitch(r,aim.pitch));CyDirection now;check(r,cy_gun_aim_now(r,&now));double delta;check(r,cy_angles_angle_diff(aim.heading,now.heading,&delta));int canFire;check(r,cy_gun_can_fire(r,power,10,&canFire));
            if(abs(delta)<3&&abs(aim.pitch-now.pitch)<3&&canFire!=0){check(r,cy_gun_fire(r));check(r,cy_gun_power(r,power));}show(r,"TRACK",target.range);
        }else if(cy_tracker_cue(r,&cue)==CY_OK){
            double bearing,distance,elevation=0,arc;check(r,cy_angles_bearing_to(&position,&cue.position,&bearing));check(r,cy_angles_distance(&position,&cue.position,&distance));if(freeFlight)check(r,cy_angles_elevation_to(&position,&cue.position,&elevation));
            const error=min(cue.bearing_error,30),across=(tick%5-2)/2.0*error,up=freeFlight?((tick/5)%3-1)*error/2:0;
            drive(r,0.7,steer(bearing,heading),steer(elevation,pitch));check(r,cy_scanner_arc_for_reach(r,distance+100,&arc));scan(r,bearing+across,arc,elevation+up);show(r,"CUE",distance);
        }else{
            double sweep;check(r,cy_angles_normalize(last.heading+36,&sweep));drive(r,0.5,0.33,steer(0,pitch));scan(r,sweep,40,freeFlight?((tick/10)%3-1)*35:0);show(r,"SEARCH");
        }
        if(get!cy_battery_charge(r)<get!cy_battery_capacity(r)/4)check(r,cy_drive_throttle(r,get!cy_drive_get_throttle(r)*0.5));
        CyStorm storm;double margin;if(cy_nav_storm(r,&storm)==CY_OK&&cy_nav_storm_margin(r,&position,&margin)==CY_OK&&margin>-60){double away,delta,control;check(r,cy_angles_bearing_to(&storm.center,&position,&away));check(r,cy_angles_angle_diff(away,heading,&delta));check(r,cy_angles_clamp(delta/30,-1,1,&control));check(r,cy_drive_throttle(r,0.8));check(r,cy_drive_turn(r,control));}
        CyZone zone;if(cy_nav_zone(r,&zone)==CY_OK&&cy_nav_zone_margin(r,&position,&margin)==CY_OK&&margin<60){double bearing,climb=0,delta,turn,pitchTurn;check(r,cy_angles_bearing_to(&position,&zone.center,&bearing));if(zone.shape==CY_SHAPE_SPHERE)check(r,cy_angles_elevation_to(&position,&zone.center,&climb));check(r,cy_angles_angle_diff(bearing,heading,&delta));check(r,cy_angles_clamp(delta/30,-1,1,&turn));check(r,cy_angles_clamp((climb-pitch)/30,-1,1,&pitchTurn));drive(r,0.8,turn,pitchTurn);}
        // Only the arena's rate calibration sends this; safe to delete.
        double work;enum path="state.calibration.work";if(cy_raw_number(r,path.ptr,path.length,&work)==CY_OK&&work>=1)show(r,"WORK",calibrationWork(cast(ulong)work,cast(ulong)tick));
    }
}
void main(){auto robot=new Starter;run(&robot.turn,&robot.start);}
