#include "state-machine.h"

input_struct::input_struct(uint_fast32_t *run_time_pointer)
{
    run_time = run_time_pointer;
}
void input_struct::receive_state(bool new_state)
{
    state = new_state;
    if (new_state != prev_state)
        input_change_time = *run_time;
}
void input_struct::receive_value(float new_value)
{
    value = new_value;
    if (new_value != prev_value)
        input_change_time = *run_time;
}
uint_fast32_t input_struct::seconds_since_change()
{
    return *run_time - input_change_time;
}
bool input_struct::has_flag()
{
    return (prev_value != value || prev_state != state);
}
void input_struct::unflag()
{
    prev_state = state;
    prev_value = value;
}
state_machine_class::state_machine_class()
{
    // initialize the list with inputs
    input[THERMOSTAT] = new input_struct(&run_time_value);
    input[THERMOSTAT_SENSOR] = new input_struct(&run_time_value);
    input[COMPRESSOR] = new input_struct(&run_time_value);
    input[SWW_RUN] = new input_struct(&run_time_value);
    input[DEFROST_RUN] = new input_struct(&run_time_value);
    input[OAT] = new input_struct(&run_time_value);
    input[STOOKLIJN_TARGET] = new input_struct(&run_time_value);
    input[TRACKING_VALUE] = new input_struct(&run_time_value);
    input[BOOST] = new input_struct(&run_time_value);
    input[BACKUP_HEAT] = new input_struct(&run_time_value);
    input[EXTERNAL_PUMP] = new input_struct(&run_time_value);
    input[RELAY_HEAT] = new input_struct(&run_time_value);
    input[TEMP_NEW_TARGET] = new input_struct(&run_time_value);
    input[WP_PUMP] = new input_struct(&run_time_value);
    input[SILENT_MODE] = new input_struct(&run_time_value);
    input[EMERGENCY] = new input_struct(&run_time_value);
}
state_machine_class::~state_machine_class()
{
    delete input[THERMOSTAT];
    delete input[THERMOSTAT_SENSOR];
    delete input[COMPRESSOR];
    delete input[SWW_RUN];
    delete input[DEFROST_RUN];
    delete input[OAT];
    delete input[STOOKLIJN_TARGET];
    delete input[TRACKING_VALUE];
    delete input[BOOST];
    delete input[BACKUP_HEAT];
    delete input[EXTERNAL_PUMP];
    delete input[RELAY_HEAT];
    delete input[TEMP_NEW_TARGET];
    delete input[WP_PUMP];
    delete input[SILENT_MODE];
    delete input[EMERGENCY];
}
void state_machine_class::run_cycle()
{
    // do cycle logic in here
    // State machine, main algoritm that runs every 'clock' cycle
    // INIT: Initial startup (very early after boot)
    // IDLE: No heat request, wait for thermostat on
    // START: Set values and start HP
    // STARTING: Early start, temporary state (waiting for compressor start)
    // STABILIZE: Early in the run < 20 minutes of compressor time or not stable yet
    // RUN: HP operating with target = stooklijn_target (+/- 1 ) and no intervention. Decides on intevention (STALL, OVERSHOOT)
    // OVERSHOOT: Raising target above stooklijn_target to prevent compressor stop, should return to RUN when done
    // STALL: Temperature below target, returns to RUN when within limits
    // WAIT: Compressor has stopped during run with still heating request (so algoritm failed)
    // SWW: HP operating to create hot water
    // DEFROST: HP operating defrost cycle
    // AFTERRUN: Run done (no more heating request) external pump runs

    static uint_fast32_t dt = 30; // round(id(state_machine).get_update_interval()/1000); //update interval in seconds
    fsm.increment_run_time(dt);   // increment fsm run_time

    //***************************************************************
    //*******************INITIALIZE RUN******************************
    //***************************************************************
    // do not run this until INIT is finished
    if (fsm.state() != INIT)
    {
        // Receive all inputs
        fsm.receive_inputs();
        fsm.process_inputs();
    }

    //***************************************************************
    //*******************State Machine States************************
    //***************************************************************
    // process state
    switch (fsm.state())
    {
    case INIT:
    {
        // DESCRIPTION: Early start. Wait for 1 minute to allow all sensor values to populate. Has 'instant on' mode to bypass some checks
        // INTERPRETS INPUTS: THERMOSTAT_SENSOR (for instant on)
        // RECEIVES EVENTS: none
        // STATE TRANSITIONS: START; IDLE
        // ENFORCE CONFIG: BACKUP_HEAT OFF; BOOST OFF
        // SPECIAL: reads raw values to determine if setup is complete
        // entry actions
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce config
        fsm.backup_heat(false);
        fsm.boost(false);
        // wait for timeout
        if (fsm.get_run_time() < 90 || isnan(id(buiten_temp).state) || isnan(id(water_temp_aanvoer).state) || isnan(id(water_temp_retour).state))
            break;
        // after timeout
        fsm.receive_inputs();
        // check for fast_start
        // the 3 places where thermostat event does not lead to a switch off. Therefore not handled through check_change_events
        if (fsm.input[THERMOSTAT]->state)
        {
            fsm.state_transition(START);
            id(controller_info).publish_state("Init complete. First state: START");
        }
        else
        {
            fsm.state_transition(IDLE);
            id(controller_info).publish_state("Init complete. First state: IDLE");
        }
        ESP_LOGD(fsm.state_name(), "INIT Complete first state: %s", fsm.state_name(fsm.get_next_state()));
        break;
    }
    case IDLE:
    {
        // DESCRIPTION: Does nothing until thermostat has a signal (after input delay)
        // INTERPRETS INPUTS: None
        // RECEIVES EVENTS: THERMOSTAT ON, SWW_RUN
        // STATE TRANSITIONS: START;SWW;DEFROST
        // ENFORCE CONFIG: BOOST OFF; BACKUP_HEAT OFF; EXTERNAL_PUMP OFF; RELAY_HEAT OFF;
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce config
        fsm.backup_heat(false);
        fsm.heat(false);
        fsm.external_pump(false);
        fsm.boost(false);

        // check events
        fsm.start_events();
        fsm.add_event(SWW_RUN);
        fsm.add_event(DEFROST_RUN);
        if (fsm.check_change_events())
            break;
        // the 3 places where thermostat event does not lead to a switch off. Therefore not handled through check_change_events
        if (fsm.input[THERMOSTAT]->state)
        {
            fsm.state_transition(START);
            ESP_LOGD(fsm.state_name(), "THERMOSTAT ON next state: START");
        }
        break;
    }
    case START:
    {
        // DESCRIPTION: Transient state, sets initial values and passes through to STARTING
        // INTERPRETS INPUTS: NONE
        // RECEIVES EVENTS: NONE
        // STATE TRANSITIONS: STARTING
        // ENFORCE CONFIG: BACKUP_HEAT OFF
        // SPECIAL: NONE
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
            fsm.external_pump(true); // external pump on
            fsm.heat(true);          // heat on (to start heatpump)
            fsm.backup_heat(false);
        }
        // three minutes delay to allow pump to run and values to stabilise
        if (fsm.seconds_since_state_start() < (3 * 60))
            break;
        // set target, with minimum of tracking value+2 (to ensure compressor start)
        // but not above stooklijn_target
        int new_target = fsm.input[STOOKLIJN_TARGET]->value + fsm.get_target_offset();
        if (new_target < fsm.input[TRACKING_VALUE]->value + 2)
            new_target = fsm.input[TRACKING_VALUE]->value + 2;
        if (new_target > fsm.input[STOOKLIJN_TARGET]->value)
            new_target = fsm.input[STOOKLIJN_TARGET]->value;
        fsm.set_new_target(new_target);
        ESP_LOGD(fsm.state_name(), "Run start initial target set; stooklijn_target: %f pendel_target: %f tracking_value: %f ", fsm.input[STOOKLIJN_TARGET]->value, fsm.input[TEMP_NEW_TARGET]->value, fsm.input[TRACKING_VALUE]->value);
        fsm.set_run_start_time();
        fsm.state_transition(STARTING);
        // enforce config
        fsm.backup_heat(false);
        break;
    }
    case STARTING:
    {
        // DESCRIPTION: Transient state, switch on system and wait for compressor to start
        // INTERPRETS INPUTS: NONE
        // RECEIVES EVENTS: SWW_RUN; DEFROST_RUN; THERMOSTAT OFF; RELAY_HEAT OFF; COMPRESSOR ON
        // STATE TRANSITIONS: STABILIZE; SWW; DEFROST; AFTERRUN
        // ENFORCE CONFIG: BACKUP_HEAT OFF; BOOST OFF
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce allowed config
        fsm.backup_heat(false);
        fsm.boost(false);

        fsm.start_events();
        fsm.add_event(SWW_RUN);
        fsm.add_event(DEFROST_RUN);
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        if (fsm.check_change_events())
            break;

        if (fsm.input[COMPRESSOR]->state)
        {
            // we have ignition
            fsm.state_transition(STABILIZE);
        }
        break;
    }
    case STABILIZE:
    {
        // DESCRIPTION: Transient state, wait for temperatures to stabilize then call run
        // INTERPRETS INPUTS: NONE
        // RECEIVES EVENTS: SWW_RUN; DEFROST; THERMOSTAT OFF; RELAY_HEAT OFF; COMPRESSOR OFF ; EMERGENCY
        // STATE TRANSITIONS: RUN; WAIT; SWW; DEFROST; AFTERRUN
        // ENFORCE CONFIG: BACKUP_HEAT OFF; BOOST OFF
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce allowed config
        fsm.backup_heat(false);
        fsm.boost(false);
        fsm.start_events();
        fsm.add_event(SWW_RUN);
        fsm.add_event(DEFROST_RUN);
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        fsm.add_event(COMPRESSOR);
        if (fsm.check_change_events())
            break;

        // check how far we are in the run
        if ((fsm.get_run_time() - fsm.get_run_start_time()) > (15 * 60) || ((fsm.get_run_time() - fsm.get_run_start_time()) > (6 * 60) && fsm.compressor_modulation()))
        {
            // monitor situation
            // we are stable if derivative => -3 and <= 3 (1 degree in 20 minutes) or if compressor starts modulation (after 6 minutes)
            if (fsm.compressor_modulation() || ((fsm.derivative_D_10 * 60) >= -3 && (fsm.derivative_D_10 * 60) <= 3))
            {
                // hand over to run algoritm, run will decide on overshoot/undershoot depending on where we stabilized
                ESP_LOGD(fsm.state_name(), "Stabilized, RUN is next");
                fsm.state_transition(RUN);
                break;
            }
        }
        // else still early run
        // update target if tracking_value or stooklijn_target changed. No advanced modulation as this is useless during early run
        // limit number of updates to once every 5 minutes, unless run will be killed

        if (fsm.pendel_delta >= fsm.hysteresis || fsm.input[TEMP_NEW_TARGET]->seconds_since_change() > (5 * 60))
        {
            if (fsm.delta > 0)
            {
                fsm.input[TEMP_NEW_TARGET]->receive_value(max(fsm.input[STOOKLIJN_TARGET]->value + fsm.get_target_offset(), fsm.input[TRACKING_VALUE]->value - 4));
                fsm.input[TEMP_NEW_TARGET]->receive_value(min(fsm.input[TEMP_NEW_TARGET]->value, fsm.input[STOOKLIJN_TARGET]->value + fsm.max_overshoot));
            }
        }
        break;
    }
    case RUN:
    {
        // DESCRIPTION: Run maintains a stable run and escallates to overshoot/stall when needed
        // INTERPRETS INPUTS: DELTA; PREDICTIONS
        // RECEIVES EVENTS: SWW_RUN; DEFROST; THERMOSTAT OFF; RELAY_HEAT OFF; COMPRESSOR OFF ; EMERGENCY;
        // STATE TRANSITIONS: WAIT; SWW; DEFROST; AFTERRUN; OVERSHOOT; STALL
        // ENFORCE CONFIG: NONE
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce allowed config and check events
        fsm.start_events();
        fsm.add_event(SWW_RUN);
        fsm.add_event(DEFROST_RUN);
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        fsm.add_event(COMPRESSOR);
        fsm.add_event(EMERGENCY);
        fsm.add_event(BACKUP_HEAT);
        if (fsm.check_change_events())
            break;

        // check low TEMP (for backup_heat_always_on)
        if (fsm.check_low_temp_trigger() && fsm.input[BACKUP_HEAT]->seconds_since_change() > (15 * 60))
        {
            fsm.backup_heat(true, true);
        }

        // check if we are running on the actual target
        if (fsm.input[TEMP_NEW_TARGET]->value != fsm.input[STOOKLIJN_TARGET]->value)
        {
            // target changed, or stabilized on a different target
            if (fsm.input[TEMP_NEW_TARGET]->value < fsm.input[STOOKLIJN_TARGET]->value)
            {
                ESP_LOGD(fsm.state_name(), "Not running on stooklijn_target: new state will be stall");
                fsm.state_transition(STALL);
                break;
            }
            else
            {
                ESP_LOGD(fsm.state_name(), "Not running on stooklijn_target: new state will be overshoot");
                fsm.state_transition(OVERSHOOT);
                break;
            }
        }

        // when we are here, it means we where in a stable condition running on stooklijn_target
        // check if overshooting predicted, or if operating > 2 degrees below target (stall)
        // check predicted delta to reach in 20 minutes (pred_20_delta_5 and pred_20_delta_10)
        // then check if we have been in the current state for at least 5 minutes (to prevent over control)
        if (fsm.seconds_since_state_start() < (5 * 60))
            break;
        // then check the predicted overshoot
        if (fsm.delta >= 1 && (fsm.pred_20_delta_5 >= 2.5 || fsm.pred_20_delta_10 >= 2.5))
        {
            // start overshooting algoritm to bring temperature back
            ESP_LOGD(fsm.state_name(), "New state will be overshoot. target: %f stooklijn_target: %f delta: %f pred_20_delta_5: %f pred_20_delta_10: %f", fsm.input[TEMP_NEW_TARGET]->value, fsm.input[STOOKLIJN_TARGET]->value, fsm.delta, fsm.pred_20_delta_5, fsm.pred_20_delta_10);
            fsm.state_transition(OVERSHOOT);
            break;
        }
        else if (fsm.delta <= -2 || (fsm.delta <= -1 && (fsm.pred_20_delta_5 < -3 || fsm.pred_20_delta_10 < -3)))
        {
            // stall, or stall predicted
            ESP_LOGD(fsm.state_name(), "New state will be stall. target: %f stooklijn_target: %f delta: %f pred_20_delta_5: %f pred_20_delta_10: %f", fsm.input[TEMP_NEW_TARGET]->value, fsm.input[STOOKLIJN_TARGET]->value, fsm.delta, fsm.pred_20_delta_5, fsm.pred_20_delta_10);
            fsm.state_transition(STALL);
            break;
        } // else status quo
        break;
    }
    case OVERSHOOT:
    {
        // DESCRIPTION: Logic to contain overshoot and return back to target
        // INTERPRETS INPUTS: DELTA; PREDICTIONS
        // RECEIVES EVENTS: SWW_RUN; DEFROST; THERMOSTAT OFF; RELAY_HEAT OFF; COMPRESSOR OFF
        // STATE TRANSITIONS: RUN; WAIT; SWW; DEFROST; AFTERRUN
        // ENFORCE CONFIG: BACKUP_HEAT OFF
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce allowed config and check events
        fsm.backup_heat(false);
        fsm.start_events();
        fsm.add_event(SWW_RUN);
        fsm.add_event(DEFROST_RUN);
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        fsm.add_event(COMPRESSOR);
        if (fsm.check_change_events())
            break;

        if (fsm.delta < 1 && fsm.pred_20_delta_5 < 1.5 && fsm.pred_20_delta_10 < 1.5)
        {
            // delta within range, are we done?
            if (fsm.input[TEMP_NEW_TARGET]->value <= fsm.input[STOOKLIJN_TARGET]->value)
            {
                // overshoot contained operating below or at target
                // hand back to RUN at target
                fsm.input[TEMP_NEW_TARGET]->receive_value(fsm.input[STOOKLIJN_TARGET]->value);
                ESP_LOGD(fsm.state_name(), "stooklijn_target <= pendel_target, delta < 2, no overshoot predicted, my job is done.");
                fsm.state_transition(RUN);
                break;
            }
        }
        if (fsm.pendel_delta >= fsm.hysteresis)
        {
            // emergency situation, run is about to be killed. Raise Target to prevent
            fsm.input[TEMP_NEW_TARGET]->receive_value(min(fsm.input[TEMP_NEW_TARGET]->value + 1, fsm.input[STOOKLIJN_TARGET]->value + fsm.max_overshoot));
            ESP_LOGD(fsm.state_name(), "Emergency intervention, raised pendel_target (%f) (if there was room)", fsm.input[TEMP_NEW_TARGET]->value);
            break;
        }
        if (fsm.input[TEMP_NEW_TARGET]->value > fsm.input[STOOKLIJN_TARGET]->value)
        {
            // target overshoot logic to return to target
            // check if target can be lowered without killing the run
            if (fsm.pendel_delta <= fsm.hysteresis - 1)
            {
                // lower target, but not below fsm.input[STOOKLIJN_TARGET]->value next step may do that if needed
                fsm.input[TEMP_NEW_TARGET]->receive_value(max(fsm.input[STOOKLIJN_TARGET]->value, fsm.input[TEMP_NEW_TARGET]->value - 1));
                ESP_LOGD(fsm.state_name(), "Operating above stooklijn_target pendel_target (%f) could be lowered", fsm.input[TEMP_NEW_TARGET]->value);
                break;
            }
        }
        ESP_LOGD(fsm.state_name(), "waiting for (predicted)delta to come within rage delta: %f, pred_20_delta_5: %f, pred_20_delta_10: %f", fsm.delta, fsm.pred_20_delta_5, fsm.pred_20_delta_10);
        break;
    }
    case STALL:
    {
        // Stall! Stall! Stall, I have control
        // DESCRIPTION: Logic to raise target to return back to stooklijn_target (and hopefully prevent overshoot)
        // INTERPRETS INPUTS: DELTA; PREDICTIONS
        // RECEIVES EVENTS: SWW_RUN; DEFROST; THERMOSTAT OFF; RELAY_HEAT OFF; COMPRESSOR OFF; EMERGENCY
        // STATE TRANSITIONS: RUN; WAIT; SWW; DEFROST; AFTERRUN
        // ENFORCE CONFIG: NONE
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce allowed config
        fsm.start_events();
        fsm.add_event(SWW_RUN);
        fsm.add_event(DEFROST_RUN);
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        fsm.add_event(COMPRESSOR);
        fsm.add_event(EMERGENCY);
        fsm.add_event(BACKUP_HEAT);
        if (fsm.check_change_events())
            break;

        // check low temp (for backup_heat_always_on)
        if (fsm.check_low_temp_trigger() && fsm.input[BACKUP_HEAT]->seconds_since_change() > (15 * 60))
        {
            fsm.backup_heat(true, true);
        }

        // 1: check if recovered
        if (fsm.input[TEMP_NEW_TARGET]->value >= fsm.input[STOOKLIJN_TARGET]->value && fsm.delta >= 0 && fsm.pred_20_delta_5 >= 0 && fsm.pred_20_delta_10 >= 0)
        {
            // target is no longer below stooklijn_target. No longer a stall
            // return to target and call run
            fsm.input[TEMP_NEW_TARGET]->receive_value(fsm.input[STOOKLIJN_TARGET]->value);
            ESP_LOGD(fsm.state_name(), "delta > 0, stooklijn_target >= pendel_target, my job is done.");
            fsm.state_transition(RUN);
            break;
        }

        // 2: check if below stooklijn target, with delta > 0 and modulating
        //  (usually target change (boost) or after start). No minimum waiting time
        if (fsm.input[TEMP_NEW_TARGET]->value < fsm.input[STOOKLIJN_TARGET]->value && fsm.delta > 0 && fsm.compressor_modulation())
        {
            fsm.input[TEMP_NEW_TARGET]->receive_value(fsm.input[STOOKLIJN_TARGET]->value);
            break;
        }

        // otherwise always at least 10 minutes waiting time
        if (fsm.input[TEMP_NEW_TARGET]->seconds_since_change() < (10 * 60))
        {
            ESP_LOGD(fsm.state_name(), "Stall is waiting for effect of previous target change");
            break;
        }

        // 3: check if operating below stooklijn_target and fix it
        if (fsm.input[TEMP_NEW_TARGET]->value < fsm.input[STOOKLIJN_TARGET]->value)
        {
            // is it bad?
            if ((fsm.delta + (fsm.derivative_D_5 * 30)) < 0)
            {
                // it will not be fixed next 30 minutes, take a big step
                // current target + 3 or tracking value, whichever is higher
                fsm.input[TEMP_NEW_TARGET]->receive_value(max(fsm.input[TRACKING_VALUE]->value, fsm.input[TEMP_NEW_TARGET]->value + 3));
            }
            else
            {
                // current target + 1 or tracking value, whichever is higher
                fsm.input[TEMP_NEW_TARGET]->receive_value(max(fsm.input[TRACKING_VALUE]->value, fsm.input[TEMP_NEW_TARGET]->value + 1));
            }
            // but not above stooklijn_target (yet)
            fsm.input[TEMP_NEW_TARGET]->receive_value(min(fsm.input[STOOKLIJN_TARGET]->value, fsm.input[TEMP_NEW_TARGET]->value));
            ESP_LOGD(fsm.state_name(), "Operating below target, raising target, pendel_target: %f", fsm.input[TEMP_NEW_TARGET]->value);
            break;
        }
        // 4: We are operating at target, are we modulating?
        if (fsm.compressor_modulation() && fsm.input[TEMP_NEW_TARGET]->value < fsm.input[STOOKLIJN_TARGET]->value + 3)
        {
            // raise target above stooklijn target to stop modulation
            fsm.input[TEMP_NEW_TARGET]->receive_value(min(fsm.input[STOOKLIJN_TARGET]->value + 3, fsm.input[TRACKING_VALUE]->value + 3));
            ESP_LOGD(fsm.state_name(), "Modulating, raising target, pendel_target: %f", fsm.input[TEMP_NEW_TARGET]->value);
            break;
        }
        // 5 We are above target and with no modulation, so those tricks are gone. How bad is it?
        if ((fsm.delta + (fsm.derivative_D_5 * 30)) < 0)
        {
            // it will still not be fixed next 30 minutes
            if (fsm.input[OAT]->value < id(backup_heater_active_temp).state && !id(relay_backup_heat).state)
            {
                id(relay_backup_heat).turn_on();
                ESP_LOGD(fsm.state_name(), "tracking_value stalled, switched backup_heater on");
            }
            break;
        }
        // Waiting for delta te become within range
        ESP_LOGD(fsm.state_name(), "Stall is waiting for next action (or out of options).");
        break;
    }
    case WAIT:
    {
        // DESCRIPTION: Failed run? The compressor has stopped, but the thermostat is still requesting heat...
        // INTERPRETS INPUTS: NONE
        // RECEIVES EVENTS: SWW_RUN; DEFROST; THERMOSTAT OFF; RELAY_HEAT OFF; COMPRESSOR ON
        // STATE TRANSITIONS: RUN; SWW; DEFROST; AFTERRUN
        // ENFORCE CONFIG: NONE
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // enforce allowed config
        fsm.start_events();
        fsm.add_event(SWW_RUN);
        fsm.add_event(DEFROST_RUN);
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        fsm.add_event(BACKUP_HEAT);
        if (fsm.check_change_events())
            break;

        // check if stooklijn value changed
        if (fsm.input[STOOKLIJN_TARGET]->has_flag())
        {
            fsm.input[TEMP_NEW_TARGET]->receive_value(fsm.input[STOOKLIJN_TARGET]->value);
            ESP_LOGD(fsm.state_name(), "Target changed: Setting new target: %f", fsm.input[TEMP_NEW_TARGET]->value);
        }
        // wait at least 6 minutes before switching to run, even if compressor is running
        if (fsm.seconds_since_state_start() < (6 * 60))
            break;
        if (fsm.input[COMPRESSOR]->state)
        {
            fsm.state_transition(RUN);
            break;
        }
        break;
    }
    case SWW:
    {
        // DESCRIPTION: SWW RUN. Monitor and decide on next state
        // INTERPRETS INPUTS: NONE
        // RECEIVES EVENTS: DEFROST; THERMOSTAT OFF; RELAY_HEAT OFF;
        // STATE TRANSITIONS: RUN; WAIT; DEFROST; AFTERRUN
        // ENFORCE CONFIG: NONE
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
            if (fsm.input[THERMOSTAT]->state && fsm.input[OAT]->value <= id(backup_heater_active_temp).state)
            {
                fsm.backup_heat(true);
            }
            else
                id(controller_info).publish_state("Starting SWW with no backup heat.");
        }
        // enforce allowed config
        fsm.start_events();
        fsm.add_event(DEFROST_RUN);
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        fsm.add_event(BACKUP_HEAT);
        if (fsm.check_change_events())
            break;
        if (fsm.input[THERMOSTAT]->has_flag() && fsm.input[THERMOSTAT]->state)
        {
            if (fsm.input[OAT]->value <= id(backup_heater_active_temp).state)
            {
                fsm.backup_heat(true);
                id(controller_info).publish_state("SWW thermostat on: backup heat on");
            }
        }
        if (!fsm.input[SWW_RUN]->state)
        {
            // end of SWW run
            if (!fsm.input[THERMOSTAT_SENSOR]->state)
            {
                // straight off if no thermostat signal after SWW (ignore delay)
                fsm.state_transition(AFTERRUN);
                break;
            }
            else
            {
                if (fsm.input[COMPRESSOR]->state)
                    fsm.state_transition(RUN);
                else
                    fsm.state_transition(WAIT);
                // start boost if we were running without backup heat
                if (!fsm.input[BACKUP_HEAT]->state)
                {
                    if (fsm.input[OAT]->value > id(backup_heater_active_temp).state)
                        fsm.boost(true);
                    id(controller_info).publish_state("SWW done starting boost.");
                    fsm.boost(true);
                }
                fsm.input[TEMP_NEW_TARGET]->receive_value(fsm.input[STOOKLIJN_TARGET]->value);
                break;
            }
        }
        break;
    }
    case DEFROST:
    {
        // DESCRIPTION: DEFROST RUN. Monitor and decide on next state
        // INTERPRETS INPUTS: NONE
        // RECEIVES EVENTS: NONE;
        // STATE TRANSITIONS: RUN; WAIT; SWW; AFTERRUN
        // ENFORCE CONFIG: NONE
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
            if (fsm.input[THERMOSTAT]->state && fsm.input[OAT]->value <= id(backup_heater_active_temp).state)
            {
                fsm.backup_heat(true);
            }
            else
                id(controller_info).publish_state("DEFROST with backup heat off.");
        }
        // enforce allowed config
        fsm.start_events();
        fsm.add_event(THERMOSTAT);
        fsm.add_event(RELAY_HEAT);
        fsm.add_event(BACKUP_HEAT);
        if (fsm.check_change_events())
            break;

        if (!fsm.input[DEFROST_RUN]->state)
        {
            // defrosting stopped initially start with stooklijn_target as target
            if (fsm.input[TEMP_NEW_TARGET]->value != fsm.input[STOOKLIJN_TARGET]->value)
                fsm.input[TEMP_NEW_TARGET]->receive_value(fsm.input[STOOKLIJN_TARGET]->value);
            // 10 minute delay (defrost takes 4 minutes) some additional delay to allow values to stabilize and backup heater to run
            if (fsm.seconds_since_state_start() < (10 * 60))
                break;
            if (!fsm.input[THERMOSTAT_SENSOR]->state)
            {
                // straight off if no thermostat signal after SWW (ignore delay)
                fsm.state_transition(AFTERRUN);
                break;
            }
            if (fsm.input[COMPRESSOR]->state)
            {
                if (fsm.delta > 0)
                {
                    fsm.backup_heat(false);
                    fsm.state_transition(RUN);
                }
                else
                {
                    fsm.state_transition(STALL);
                }
            }
            else
            {
                fsm.backup_heat(false);
                fsm.state_transition(WAIT);
            }
        }
        break;
    }
    case AFTERRUN:
    {
        // DESCRIPTION: Shutdown and let pump run for x minutes
        // INTERPRETS INPUTS: NONE
        // RECEIVES EVENTS: THERMOSTAT; SWW_RUN
        // STATE TRANSITIONS: IDLE; sww
        // ENFORCE CONFIG: BACKUP_HEAT OFF; RELAY_HEAT OFF; BOOST_OFF
        // SPECIAL: none
        if (!fsm.entry_done)
        {
            fsm.entry_done = true;
        }
        // check events and enforce config
        fsm.backup_heat(false);
        fsm.heat(false);
        fsm.boost(false);
        fsm.start_events();
        fsm.add_event(SWW_RUN);
        if (fsm.check_change_events())
            break;
        // the 3 places where thermostat event does not lead to a switch off. Therefore not handled through check_change_events
        if (fsm.input[THERMOSTAT]->state)
        {
            fsm.state_transition(START);
            ESP_LOGD(fsm.state_name(), "THERMOSTAT ON next state: START");
        }
        // Timeout
        if (fsm.seconds_since_state_start() < (id(external_pump_runover).state * 60))
            break;
        fsm.state_transition(IDLE);
        break;
    }
    case NONE:
    {
        ESP_LOGE(fsm.state_name(), "ERROR: State is none");
        id(controller_info).publish_state("ERROR: state = NONE");
    }
    }

    if (fsm.get_run_time() % fsm.alive_timer == 0)
    {
        ESP_LOGD(fsm.state_name(), "**alive** timer: %d oat: %f inlet: %f outlet: %f tracking_value: %f stooklijn: %f pendel: %f delta: %f pendel_delta: %f ", fsm.get_run_time(), fsm.input[OAT]->value, id(water_temp_retour).state, id(water_temp_aanvoer).state, fsm.input[TRACKING_VALUE]->value, fsm.input[STOOKLIJN_TARGET]->value, fsm.input[TEMP_NEW_TARGET]->value, fsm.delta, fsm.pendel_delta);
    }

    //***************************************************************
    //*******************Post Run Cleanup****************************
    //***************************************************************
    // Update modbus target if temp_new_target has changed
    if (fsm.input[TEMP_NEW_TARGET]->has_flag() && fsm.input[TEMP_NEW_TARGET]->value != (float)id(doel_temp).state && fsm.state() != INIT)
    {
        // prevent update while still in INIT
        // Update target through modbus
        fsm.set_target_temp(fsm.input[TEMP_NEW_TARGET]->value);
    }

    // Now unflag all input values to be able to track changes on next run
    fsm.unflag_input_values();
    // Complete state transition that was initiated
    fsm.handle_state_transition();
}
void state_machine_class::update_stooklijn()
{
    update_stooklijn_bool = true;
}
states state_machine_class::state()
{
    return current_state;
}
states state_machine_class::get_prev_state()
{
    return prev_state;
}
states state_machine_class::get_next_state()
{
    return next_state;
}
void state_machine_class::state_transition(states newstate)
{
    next_state = newstate;
    ESP_LOGD(state_name(), "State transition-> %s", state_name(get_next_state()));
}
void state_machine_class::handle_state_transition()
{
    if (get_next_state() != state() && get_next_state() != NONE)
    {
        prev_state = current_state;
        current_state = get_next_state();
        state_start_time = get_run_time();
        entry_done = false;
        id(controller_state).publish_state(state_name());
        ESP_LOGD(state_name(), "State transition complete-> %s", state_name());
    }
}
const char *state_machine_class::state_friendly_name(states stt)
{
    if (stt == NONE)
        stt = current_state;
    static const std::string state_string_friendly_list[13] = {"None", "Initialiseren", "Uit", "Start", "Opstarten", "Aan (stabiliseren)", "Aan (verwarmen)", "Aan (overshoot)", "Aan (stall)", "Pauze (Uit)", "Aan (Warm Water)", "Ontdooien", "Nadraaien"};
    return state_string_friendly_list[stt].c_str();
}
const char *state_machine_class::state_name(states stt)
{
    if (stt == NONE)
        stt = current_state;
    static const std::string state_string_list[13] = {"NONE", "INIT", "IDLE", "START", "STARTING", "STABILIZE", "RUN", "OVERSHOOT", "STALL", "WAIT", "SWW", "DEFROST", "AFTERRUN"};
    return state_string_list[stt].c_str();
}
uint_fast32_t state_machine_class::get_run_time()
{
    return run_time_value;
}
void state_machine_class::increment_run_time(uint_fast32_t increment)
{
    run_time_value += increment;
}
void state_machine_class::set_run_start_time()
{
    run_start_time = get_run_time();
}
uint_fast32_t state_machine_class::get_run_start_time()
{
    return run_start_time;
}
uint_fast32_t state_machine_class::get_state_start_time()
{
    return state_start_time;
}
uint_fast32_t state_machine_class::seconds_since_state_start()
{
    return get_run_time() - get_state_start_time();
}
// receive all values, booleans (states) or floats (values)
void state_machine_class::receive_inputs()
{
    input[THERMOSTAT_SENSOR]->receive_state(id(thermostat_signal).state); // state of thermostat input
    input[THERMOSTAT]->receive_state(thermostat_state());
    input[COMPRESSOR]->receive_state(id(compressor_running).state); // is the compressor running
    input[SWW_RUN]->receive_state(id(sww_heating).state);           // is the domestic hot water run active
    input[DEFROST_RUN]->receive_state(id(defrosting).state);        // is defrost active
    input[OAT]->receive_value(round(id(buiten_temp).state));        // outside air temperature
    if (input[OAT]->has_flag() || update_stooklijn_bool)
        input[STOOKLIJN_TARGET]->receive_value(calculate_stooklijn()); // stooklijn target
    // Set to value that anti-pendel script will track (outlet/inlet) (recommend inlet)
    input[TRACKING_VALUE]->receive_value(floor(id(water_temp_aanvoer).state));
    input[BOOST]->receive_state(id(boost_switch).state);
    input[BACKUP_HEAT]->receive_state(id(relay_backup_heat).state); // is backup heat on/off
    input[EXTERNAL_PUMP]->receive_state(id(relay_pump).state);      // is external pump on/off
    input[RELAY_HEAT]->receive_state(id(relay_heat).state);         // is realy_heat (heatpump external thermostat contact) on/off
    input[WP_PUMP]->receive_state(id(pump_running).state);          // is internal pump running
    input[SILENT_MODE]->receive_state(id(silent_mode_state).state); // is silent mode on
    if (input[TEMP_NEW_TARGET]->value == 0.0)
        input[TEMP_NEW_TARGET]->value = input[STOOKLIJN_TARGET]->value; // set temp new target
    delta = input[TRACKING_VALUE]->value - input[STOOKLIJN_TARGET]->value;
    pendel_delta = input[TRACKING_VALUE]->value - input[TEMP_NEW_TARGET]->value;
}
void state_machine_class::process_inputs()
{
    if (input[BOOST]->state)
    {
        if (input[BOOST]->seconds_since_change() > (id(boost_time).state * 60))
            boost(false);
    }
    if (input[BOOST]->has_flag())
    {
        toggle_boost();
    }
    toggle_silent_mode();
    if (input[WP_PUMP]->state && state() != SWW && state() != DEFROST)
    {
        // calculate derivative and publish new value
        calculate_derivative(input[TRACKING_VALUE]->value);
    }
    else if (!input[WP_PUMP]->state && derivative.size() > 0)
    {
        // if pump not running and derivative has values clear it
        derivative.clear();
        id(derivative_value).publish_state(0);
    }
}
// set input.value.prev_value = input_value.value to remove the implicit 'value changed' flag
void state_machine_class::unflag_input_values()
{
    input[THERMOSTAT_SENSOR]->unflag();
    input[THERMOSTAT]->unflag();
    input[COMPRESSOR]->unflag();
    input[SWW_RUN]->unflag();
    input[DEFROST_RUN]->unflag();
    input[OAT]->unflag();
    input[STOOKLIJN_TARGET]->unflag();
    input[TRACKING_VALUE]->unflag();
    input[BOOST]->unflag();
    input[BACKUP_HEAT]->unflag();
    input[EXTERNAL_PUMP]->unflag();
    input[RELAY_HEAT]->unflag();
    input[TEMP_NEW_TARGET]->unflag();
    input[WP_PUMP]->unflag();
    input[SILENT_MODE]->unflag();
    input[EMERGENCY]->unflag();
}
//***************************************************************
//*******************Stooklijn***********************************
//***************************************************************
// calculate stooklijn function
float state_machine_class::calculate_stooklijn()
{
    // Calculate stooklijn target
    // Hold previous script run oat value
    static float prev_oat = 20; // oat at minimum water temp (20/20) to prevent strange events on startup
    // wait for a valid oat reading
    float oat = 20;
    if (input[OAT]->value > 60 || input[OAT]->value < -50 || isnan(input[OAT]->value))
    {
        oat = prev_oat;
        // use prev_oat (or 20) and do not set update_stooklijn to false, to trigger a new run on next cycle
        ESP_LOGD("calculate_stooklijn", "Invalid OAT (%f) waiting for next run", input[OAT]->value);
    }
    else
    {
        prev_oat = input[OAT]->value;
        update_stooklijn_bool = false;
    }
    float new_stooklijn_target;
    // OAT expects start temp to be OAT 20 with Watertemp 20. Steepness is defined bij Z, calculated by the max wTemp at minOat
    // Formula is wTemp = ((Z x (stooklijn_max_oat - OAT)) + stooklijn_min_wtemp) + C
    // Formula to calculate Z = 0-((stooklijn_max_wtemp-stooklijn_min_wtemp)) / (stooklijn_min_oat - stooklijn_max_oat))
    // C is the curvature of the stooklijn defined by C = (stooklijn_curve*0.001)*(oat-max_oat)^2
    // This will add a positive offset with decreasing offset. You can set this to zero if you don't need it and want a linear stooklijn
    // I need it in my installation as the stooklijn is spot on at relative high temperatures, but too low at lower temps
    const float Z = 0 - (float)((id(stooklijn_max_wtemp).state - id(stooklijn_min_wtemp).state) / (id(stooklijn_min_oat).state - id(stooklijn_max_oat).state));
    // If oat above or below maximum/minimum oat, clamp to stooklijn_max/min value
    float oat_value = input[OAT]->value;
    if (oat_value > id(stooklijn_max_oat).state)
        oat_value = id(stooklijn_max_oat).state;
    else if (oat_value < id(stooklijn_min_oat).state)
        oat_value = id(stooklijn_min_oat).state;
    float C = (id(stooklijn_curve).state * 0.001) * pow((oat_value - id(stooklijn_max_oat).state), 2);
    new_stooklijn_target = (int)round((Z * (id(stooklijn_max_oat).state - oat_value)) + id(stooklijn_min_wtemp).state + C);
    // Add stooklijn offset
    new_stooklijn_target = new_stooklijn_target + id(wp_stooklijn_offset).state;
    // Add boost offset
    new_stooklijn_target = new_stooklijn_target + current_boost_offset;
    // Clamp target to minimum temp/max water+3
    clamp(new_stooklijn_target, id(stooklijn_min_wtemp).state, id(stooklijn_max_wtemp).state + 3);
    ESP_LOGD("calculate_stooklijn", "Stooklijn calculated with oat: %f, Z: %f, C: %f offset: %f, result: %f", input[OAT]->value, Z, C, id(wp_stooklijn_offset).state, new_stooklijn_target);
    // Publish new stooklijn value to watertemp value sensor
    id(watertemp_target).publish_state(new_stooklijn_target);
    return new_stooklijn_target;
}
//***************************************************************
//*******************Thermostat**********************************
//***************************************************************
bool state_machine_class::thermostat_state()
{
    // if sensor and thermostat are the same, just return
    if (input[THERMOSTAT_SENSOR]->state == input[THERMOSTAT]->state)
        return input[THERMOSTAT]->state;
    // instant on
    if (state() == INIT && input[THERMOSTAT_SENSOR]->state)
        return true;
    // thermostat change
    if (input[THERMOSTAT_SENSOR]->state)
    {
        // state change is a switch to on
        // check if on delay has passed
        if (input[THERMOSTAT_SENSOR]->seconds_since_change() > (id(thermostat_on_delay).state * 60))
            return true;
    }
    else
    {
        // state change is a switch to off
        // check for instant off
        if (!input[COMPRESSOR]->state || state() == SWW || state() == DEFROST)
            return false;
        // check if off delay time has passed
        if (input[THERMOSTAT_SENSOR]->seconds_since_change() > (id(thermostat_off_delay).state * 60))
        {
            // then check if minimum run time has passed
            if ((get_run_time() - run_start_time) > (id(minimum_run_time).state * 60))
                return false;
        }
    }
    // return previous state
    return input[THERMOSTAT]->state;
}
//***************************************************************
//*******************Derivative**********************************
//***************************************************************
void state_machine_class::calculate_derivative(float tracking_value)
{
    float d_number = tracking_value;
    derivative.push_back(d_number);
    // limit size to 31 elements(15 minutes)
    if (derivative.size() > 31)
        derivative.erase(derivative.begin());
    // calculate current derivative for 5 and 10 minutes
    // derivative is measured in degrees/minute
    derivative_D_5 = 0;
    derivative_D_10 = 0;
    // wait until derivative > 14, this makes sure the first 2 minutes are skipped
    // first minute or so is unreliabel if pump has been off for a while (water cools in the unit)
    if (derivative.size() > 14)
    {
        derivative_D_5 = (derivative.back() - derivative.at(derivative.size() - 11)) / 10;
    }
    if (derivative.size() > 24)
    {
        derivative_D_10 = (derivative.back() - derivative.at(derivative.size() - 21)) / 20;
    }
    // make sure there is always a prediction even with derivative = 0
    pred_20_delta_5 = (tracking_value + (derivative_D_5 * 20)) - input[STOOKLIJN_TARGET]->value;
    pred_20_delta_10 = (tracking_value + (derivative_D_10 * 20)) - input[STOOKLIJN_TARGET]->value;
    pred_5_delta_5 = (tracking_value + (derivative_D_5 * 5)) - input[STOOKLIJN_TARGET]->value;
    // publish new value
    id(derivative_value).publish_state(derivative_D_10 * 60);
}
//***************************************************************
//*******************Heat****************************************
//***************************************************************
void state_machine_class::heat(bool mode)
{
    if (mode)
    {
        if (!id(relay_heat).state)
        {
            id(relay_heat).turn_on();
            input[RELAY_HEAT]->receive_state(true);
        }
        // if relay heat is turned on, relay_pump must also be turned on
        if (!input[EXTERNAL_PUMP]->state)
        {
            external_pump(true);
            ESP_LOGD(state_name(), "Invalid configuration relay_heat on before relay_pump.");
            id(controller_info).publish_state("Invalid config: heat on before pump.");
        }
    }
    else
    {
        if (id(relay_heat).state)
        {
            id(relay_heat).turn_off();
            input[RELAY_HEAT]->receive_state(false);
        }
        // external pump can remain on, backup heater must be off
        if (input[BACKUP_HEAT]->state)
        {
            backup_heat(false);
            ESP_LOGD(state_name(), "Invalid configuration relay_heat off before relay_backup_heat off.");
            id(controller_info).publish_state("Invalid config: heat off before backup_heat");
        }
    }
}
//***************************************************************
//*******************Pump****************************************
//***************************************************************
void state_machine_class::external_pump(bool mode)
{
    if (mode)
    {
        if (!id(relay_pump).state)
        {
            id(relay_pump).turn_on();
            input[EXTERNAL_PUMP]->receive_state(true);
        }
    }
    else
    {
        if (input[RELAY_HEAT]->state)
        {
            heat(false);
            ESP_LOGD(state_name(), "Invalid configuration relay_pump off before relay_heat");
            id(controller_info).publish_state("Invalid config: pump off before heat");
        }
        // backup heater must be off
        if (input[BACKUP_HEAT]->state)
        {
            backup_heat(false);
            ESP_LOGD(state_name(), "Invalid configuration relay_pump off before relay_backup_heat");
            id(controller_info).publish_state("Invalid config: pump off before backup_heat");
        }
        if (id(relay_pump).state)
        {
            id(relay_pump).turn_off();
            input[EXTERNAL_PUMP]->receive_state(false);
        }
    }
}
//***************************************************************
//*******************Backup Heat*********************************
//***************************************************************
void state_machine_class::backup_heat(bool mode, bool temp_limit_trigger)
{
    if (mode)
    {
        // relay heat must be on, otherwise it is an invalid request
        if (!input[RELAY_HEAT]->state)
        {
            // do not turn on
            ESP_LOGD(state_name(), "Invalid configuration relay_backup_heat on before relay_heat.");
            id(controller_info).publish_state("ERROR: backup_heat on before heat.");
        }
        else
        {
            if (!id(relay_backup_heat).state)
            {
                id(relay_backup_heat).turn_on();
                input[BACKUP_HEAT]->receive_state(true);
                if (temp_limit_trigger)
                {
                    backup_heat_temp_limit_trigger = true;
                    id(controller_info).publish_state("Backup heat on due to low temp");
                }
                else if (input[SWW_RUN]->state)
                {
                    id(controller_info).publish_state("Backup heat on due to SWW run");
                }
                else if (input[DEFROST_RUN]->state)
                {
                    id(controller_info).publish_state("Backup heat on due to Defrost");
                }
                else if (state() == STALL)
                {
                    id(controller_info).publish_state("Backup heat on due to STALL");
                }
                else
                {
                    id(controller_info).publish_state("Backup heat on");
                }
            }
            // if relay_backup_heat is turned on, relay_pump must also be turned on
            if (!input[EXTERNAL_PUMP]->state)
            {
                external_pump(true);
                ESP_LOGD(state_name(), "Invalid configuration relay_backup_heat on before relay_pump.");
                id(controller_info).publish_state("Invalid config: backup_heat on before pump.");
            }
            backup_heat_temp_limit_trigger = false;
        }
    }
    else
    {
        if (id(relay_backup_heat).state)
        {
            id(relay_backup_heat).turn_off();
            input[BACKUP_HEAT]->receive_state(false);
            backup_heat_temp_limit_trigger = false;
        }
        // all else can remain on
    }
}
//***************************************************************
//*******************Boost***************************************
//***************************************************************
void state_machine_class::boost(bool mode)
{
    if (mode)
    {
        if (!input[BOOST]->state)
            id(boost_switch).turn_on();
    }
    else
    {
        if (input[BOOST]->state)
            id(boost_switch).turn_off();
    }
}
void state_machine_class::toggle_boost()
{
    if (input[BOOST]->state)
    {
        current_boost_offset = boost_offset;
        input[STOOKLIJN_TARGET]->receive_value(calculate_stooklijn());
        id(controller_info).publish_state("Boost mode active");
    }
    else
    {
        current_boost_offset = 0;
        input[STOOKLIJN_TARGET]->receive_value(calculate_stooklijn());
        id(controller_info).publish_state("Boost mode deactivated");
    }
}
//***************************************************************
//*******************Silent mode logic***************************
//***************************************************************
void state_machine_class::silent_mode(bool mode)
{
    if (mode)
    {
        if (!input[SILENT_MODE]->state)
        {
            id(silent_mode_switch).turn_on();
            id(silent_mode_state).publish_state(true);
            input[SILENT_MODE]->receive_state(true);
        }
    }
    else
    {
        if (input[SILENT_MODE]->state)
        {
            id(silent_mode_switch).turn_off();
            id(silent_mode_state).publish_state(false);
            input[SILENT_MODE]->receive_state(false);
        }
    }
}
void state_machine_class::toggle_silent_mode()
{
    // if input[OAT]->value >= silent always on: silent on
    // if input[OAT]->value <= silent always off: silent off
    // if in between: if boost or stall silent off otherwise silent on

    if (input[OAT]->value >= id(oat_silent_always_on).state)
    {
        if (!input[SILENT_MODE]->state)
        {
            ESP_LOGD(state_name(), "oat > oat_silent_always_on and silent mode off, switching silent mode on");
            id(controller_info).publish_state("Switching Silent mode on oat > on");
            silent_mode(true);
        }
    }
    else if (input[OAT]->value <= id(oat_silent_always_off).state)
    {
        if (input[SILENT_MODE]->state)
        {
            ESP_LOGD(state_name(), "Oat < oat_silent_always_off Switching silent mode off");
            id(controller_info).publish_state("Switching silent mode off oat < oat_silent_always_off");
            silent_mode(false);
        }
    }
    else
    {
        if (input[BOOST]->state || state() == STALL)
        {
            if (input[SILENT_MODE]->state)
            {
                ESP_LOGD(state_name(), "OAT between silent mode brackets. Boost or stall silent mode off");
                id(controller_info).publish_state("STALL/Boost switching silent mode off");
                silent_mode(false);
            }
        }
        else if (!input[SILENT_MODE]->state)
        {
            ESP_LOGD(state_name(), "OAT between silent mode brackets. No boost/stall switching silent on");
            id(controller_info).publish_state("Switching silent mode on oat in between");
            silent_mode(true);
        }
    }
}
int state_machine_class::get_target_offset()
{
    if (input[OAT]->value >= 10)
        return -3;
    if (input[OAT]->value >= id(oat_silent_always_on).state)
        return -2;
    return -1;
}
void state_machine_class::set_new_target(float new_target)
{
    // TODO check for multiple target changes during run
    if (new_target != input[TEMP_NEW_TARGET]->value)
        input[TEMP_NEW_TARGET]->receive_value(new_target);
}
void state_machine_class::start_events()
{
    events.clear();
}
void state_machine_class::add_event(input_types ev)
{
    events.push_back(ev);
}
bool state_machine_class::check_change_events()
{
    std::vector<input_types>::iterator it;
    bool state_change = false;
    for (it = events.begin(); it != events.end(); it++)
    {
        if (*it == DEFROST_RUN)
        {
            if (input[DEFROST_RUN]->state)
            {
                state_transition(DEFROST);
                ESP_LOGD(state_name(), "DEFROST run detected next state: DEFROST");
                state_change = true;
            }
        }
        else if (*it == SWW_RUN)
        {
            if (input[SWW_RUN]->state && !input[DEFROST_RUN]->state)
            {
                state_transition(SWW);
                ESP_LOGD(state_name(), "SWW run detected next state: SWW");
                state_change = true;
            }
        }
        else if (*it == THERMOSTAT)
        {
            if (!input[THERMOSTAT]->state)
            {
                if (!input[SWW_RUN]->state && !input[DEFROST_RUN]->state)
                {
                    state_transition(AFTERRUN);
                    ESP_LOGD(state_name(), "THERMOSTAT OFF next state: AFTERRUN");
                    state_change = true;
                }
                else
                {
                    backup_heat(false);
                    heat(false);
                    state_change = false;
                }
            }
        }
        else if (*it == RELAY_HEAT)
        {
            if (!input[RELAY_HEAT]->state)
            {
                // relay_heat switched off. Check if thermostat still on (or on again)
                if (input[THERMOSTAT]->state)
                {
                    external_pump(true);
                    heat(true);
                    ESP_LOGD(state_name(), "RELAY_HEAT OFF, but thermostat_sensor on switched relay_heat back on");
                    id(controller_info).publish_state("Heat switched off; thermostat on. Heat back on");
                }
                else if (!input[SWW_RUN]->state && !input[DEFROST_RUN]->state)
                {
                    state_transition(AFTERRUN);
                    ESP_LOGD(state_name(), "RELAY_HEAT OFF next state: AFTERRUN");
                    id(controller_info).publish_state("Heat switched off. Aborting");
                    state_change = true;
                }
            }
        }
        else if (*it == COMPRESSOR)
        {
            if (!input[COMPRESSOR]->state)
            {
                // COMPRESSOR switched off. Failed run
                state_transition(WAIT);
                ESP_LOGD(state_name(), "Failed run detected next state: WAIT");
                state_change = true;
            }
        }
        else if (*it == EMERGENCY)
        {
            if (pendel_delta >= hysteresis)
            {
                state_transition(OVERSHOOT);
                state_change = true;
            }
        }
        else if (*it == BACKUP_HEAT)
        {
            if (input[BACKUP_HEAT]->state)
            {
                if (!input[RELAY_HEAT]->state || !input[THERMOSTAT]->state)
                {
                    heat(false);
                    backup_heat(false);
                    ESP_LOGD(state_name(), "Backup heat off no heat request (relay_heat off)");
                    id(controller_info).publish_state("Backup heat off due to no heat request");
                }
                else if (input[OAT]->value > id(backup_heater_active_temp).state)
                {
                    backup_heat(false);
                    ESP_LOGD(state_name(), "Backup heat off input[OAT]->value > backup_heater_active_temp");
                    id(controller_info).publish_state("Backup heat off due to high oat");
                }
                else if (backup_heat_temp_limit_trigger && input[OAT]->value > id(backup_heater_always_on_temp).state)
                {
                    // if triggered due to low temp and situation improved (with some hysteresis)
                    backup_heat(false);
                    ESP_LOGD(state_name(), "Backup heat off due to temperature improved");
                    id(controller_info).publish_state("Backup heat off due to temperature improvement");
                }
            }
        }
    }
    events.clear();
    return state_change;
}
bool state_machine_class::compressor_modulation()
{
    if (input[SILENT_MODE]->state && id(compressor_rpm).state <= 50)
        return true;
    else if (!input[SILENT_MODE]->state && id(compressor_rpm).state <= 70)
        return true;
    else
        return false;
}
bool state_machine_class::check_low_temp_trigger()
{
    return (input[OAT]->value <= id(backup_heater_always_on_temp).state);
}
// update target temp through modbus
void state_machine_class::set_target_temp(float target)
{
    auto water_temp_call = id(water_temp_target_output).make_call();
    water_temp_call.set_value(round(target));
    water_temp_call.perform();
    ESP_LOGD("set_target_temp", "Modbus target set to: %f", round(target));
    id(doel_temp).publish_state(target * 10);
}

static state_machine_class fsm;