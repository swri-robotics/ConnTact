    #State names
    # IDLE_STATE           = 'idle state'
    # CHECK_FEEDBACK_STATE = 'checking load cell feedback'
    # APPROACH_STATE       = 'approaching hole surface'
    # FIND_HOLE_STATE      = 'finding hole'
    # INSERTING_PEG_STATE  = 'inserting peg'
    # COMPLETION_STATE     = 'completed insertion'
    # SAFETY_RETRACT_STATE = 'retracing to safety' 

    #Trigger names
    # CHECK_FEEDBACK_TRIGGER     = 'check loadcell feedback'
    # START_APPROACH_TRIGGER     = 'start approach'
    # SURFACE_FOUND_TRIGGER      = 'surface found'
    # HOLE_FOUND_TRIGGER         = 'hole found'
    # ASSEMBLY_COMPLETED_TRIGGER = 'assembly completed'
    # SAFETY_RETRACTION_TRIGGER  = 'retract to safety'


class algorithm_spiral_search(PegInHoleNodeCompliance):

    def __init__(self):
        


        states = [
            IDLE_STATE,
            CHECK_FEEDBACK_STATE,
            APPROACH_STATE, 
            FIND_HOLE_STATE, 
            INSERTING_PEG_STATE, 
            COMPLETION_STATE, 
            SAFETY_RETRACT_STATE
        ]

        transitions = [
            {'trigger':CHECK_FEEDBACK_TRIGGER    , 'source':IDLE_STATE          , 'dest':CHECK_FEEDBACK_STATE, 'after': 'check_load_cell_feedback'},
            {'trigger':START_APPROACH_TRIGGER    , 'source':CHECK_FEEDBACK_STATE, 'dest':APPROACH_STATE      , 'after': 'finding_surface'         },
            {'trigger':SURFACE_FOUND_TRIGGER     , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE     , 'after': 'finding_hole'            },
            {'trigger':HOLE_FOUND_TRIGGER        , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE , 'after': 'inserting_peg'           },
            {'trigger':ASSEMBLY_COMPLETED_TRIGGER, 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE    , 'after': 'completed_insertion'     },

            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':IDLE_STATE          , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':APPROACH_STATE      , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':FIND_HOLE_STATE     , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':INSERTING_PEG_STATE , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':COMPLETION_STATE    , 'dest':SAFETY_RETRACT_STATE, 'after': 'safety_retraction'       },


        ]
        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)