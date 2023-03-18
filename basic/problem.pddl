(define (problem demo_prb)
(:domain robot-navigation)
(:objects
; locations
entrance bedroom - location ;kitchen livingroom bathroom  - location
; people
;miguel fran - person

; objects
;vaso_rojo vaso_azul vaso_verde - object

)
(:init
; the robot at start is in the entrance of the house
(rob_at bedroom)
; (person_at miguel entrance)
; (person_at fran entrance)
; (obj_at vaso_rojo bedroom)
; (obj_at vaso_azul bedroom)
; (obj_at vaso_verde bedroom)

;;;;;DEbug
(rob_outside bedroom)
(not(door_open bedroom))


(loc_connected entrance bedroom) ;Working with or gives core dump error?
(loc_connected bedroom entrance)

; (loc_connected entrance livingroom)
; (loc_connected livingroom entrance)

; (loc_connected kitchen livingroom)
; (loc_connected livingroom kitchen)

; (loc_connected bedroom bathroom)
; (loc_connected bathroom bedroom)

)
(:goal
    (and
    (rob_at entrance)

    ;(obj_at vaso_rojo entrance)
    ;(person_carry fran vaso_verde) 
    )
)
)