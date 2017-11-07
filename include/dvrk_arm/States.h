#ifndef CDVRK_StatesH
#define CDVRK_StatesH
struct States{
public:
    States(){
        stateMap[DVRK_UNINITIALIZED] = "DVRK_UNINITIALIZED";
        stateMap[DVRK_POSITION_JOINT]= "DVRK_POSITION_JOINT";
        stateMap[DVRK_POSITION_CARTESIAN] = "DVRK_POSITION_CARTESIAN";
        stateMap[DVRK_EFFORT_CARTESIAN] = "DVRK_EFFORT_CARTESIAN";


        _m_effort_mode = stateMap[DVRK_EFFORT_CARTESIAN];
        _m_jnt_pos_mode = stateMap[DVRK_POSITION_JOINT];
        _m_cart_pos_mode = stateMap[DVRK_POSITION_CARTESIAN];

        activeState = DVRK_UNINITIALIZED;
    }

    enum ARM_STATES{DVRK_UNINITIALIZED,
                DVRK_POSITION_CARTESIAN,
                DVRK_POSITION_JOINT,
                DVRK_EFFORT_CARTESIAN};
    ARM_STATES activeState;

    std::map<ARM_STATES, std::string> stateMap;

    std::string _m_effort_mode;
    std::string _m_jnt_pos_mode;
    std::string _m_cart_pos_mode;
};

#endif
