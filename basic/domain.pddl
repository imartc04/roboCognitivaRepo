(define (domain robot-navigation)
  (:requirements :typing :strips :disjunctive-preconditions :negative-preconditions)
  (:types location object person)
  (:predicates 
    (obj_at ?o - object ?l - location)
    (person_at ?p - person ?l - location)
    (rob_at ?l - location)
    (loc_connected ?l1 - location ?l2 - location)
    (rob_carry ?o - object)
    (person_carry ?p - person ?o - object)
    (door_open ?l - location)
    (rob_outside ?l - location) ; This variable defines if the robot is in near a location but just outside its door or inside the location
  )

  ;Robot moves between locations
  (:action rob_move_between_outsides
    :parameters (?from - location ?to - location)
    :precondition (and (rob_at ?from) (rob_outside ?from) (not (door_open ?from)) (loc_connected ?from ?to) )
    :effect ( and (rob_at ?to) (rob_outside ?to) (not (rob_at ?from)) (not (rob_outside ?from)) )
  )

    ;Robot close door 
    (:action rob_close_door
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (door_open ?l) ) 
        :effect (not (door_open ?l))
    
    )

    ;Robot open door 
    (:action rob_open_door
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (not(door_open ?l)) ) 
        :effect (door_open ?l)
    
    )

    ;Move outside location
    (:action rob_move_outside
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (door_open ?l) (not(rob_outside ?l)) ) 
        :effect (rob_outside ?l)

    )

    ;Move inside location
    (:action rob_move_inside
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (door_open ?l) (rob_outside ?l) ) 
        :effect (not (rob_outside ?l))
    )


;     ;Robot action to take object in a instance
;   (:action rob_take
;       :parameters (?o - object ?ol - location )
;       :precondition (and (rob_at ?ol) (not (rob_outside ?ol)) (obj_at ?o ?ol)) 
;       :effect (and (rob_carry ?o) (not (obj_at ?o ?ol))) 
;   )

;     ;Robot action to drop object in a instance
;   (:action rob_drop
;       :parameters (?o - object ?dl - location )
;       :precondition (and (rob_at ?dl) (not (rob_outside ?dl)) (rob_carry ?o))
;       :effect (and (not (rob_carry ?o)) (obj_at ?o ?dl)) 
;   )

;   ;Robot action carry object to a person
;   (:action robObjToPerson
;     :parameters (?o - object ?p - person ?rl - location)
;     :precondition (and (rob_carry ?o) (rob_at ?rl) (not (rob_outside ?rl)) (person_at ?p ?rl))
;     :effect (and (not (rob_carry ?o)) (person_carry ?p ?o)) 
;   )

;     ;Action person gives object to robot
;     (:action personObjToRob
;     :parameters (?o - object ?p - person ?rl -location )
;     :precondition (and (rob_carry ?o) (rob_at ?rl) (not (rob_outside ?rl)) (person_at ?p ?rl))
;     :effect (and (rob_carry ?o) (not (person_carry ?p ?o ))) 
;   )


)
