(define (problem demo_prb)
(:domain robot-navigation)
(:objects
; locations
kitchen bedroom livingroom bathroom entrance - location
; people
miguel fran - person

; objects
vaso_rojo vaso_azul vaso_verde - object

)
(:init
; the robot at start is in the entrance of the house
(rob_at entrance)
(person_at miguel livingroom)
(person_at fran entrance)
(obj_at vaso_rojo bedroom)
(obj_at vaso_azul livingroom)
(obj_at vaso_verde kitchen)

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
    )
)
;(rob_at kitchen))

)