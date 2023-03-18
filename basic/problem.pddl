(define (problem demo_prb)
(:domain robot-navigation)
(:objects
; locations
entrance bedroom kitchen livingroom bathroom  - location
; people
miguel fran - person

; objects
vaso_rojo vaso_azul vaso_verde - object

)
(:init
; the robot at start is in the entrance of the house
(rob_at entrance)
(rob_inside entrance)

;Init person position
(person_at miguel entrance)
(person_at fran bedroom)

;Init obj position
(obj_at vaso_rojo kitchen)
(obj_at vaso_azul livingroom)
(obj_at vaso_verde bedroom)

;Init doors
(door_closed bedroom)
(door_closed entrance)
(door_closed kitchen)
(door_closed livingroom)
(door_closed bathroom)

(loc_connected entrance bedroom) ;Working with or gives core dump error?
(loc_connected bedroom entrance)

(loc_connected entrance livingroom)
(loc_connected livingroom entrance)

(loc_connected kitchen livingroom)
(loc_connected livingroom kitchen)

(loc_connected bedroom bathroom)
(loc_connected bathroom bedroom)

)
(:goal
    (and
    
    (obj_at vaso_rojo entrance)
    (person_carry fran vaso_verde) 
    (rob_at entrance)
    (rob_inside entrance)
    )
)
)