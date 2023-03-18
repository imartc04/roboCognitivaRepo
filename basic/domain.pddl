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
    (door_closed ?l - location)
    
    (rob_outside ?l - location) ; This variable defines if the robot is in near a location but just outside its door or inside the location
    (rob_inside ?l - location)

  )

  ;Robot moves between locations
  (:action rob_move_between_outsides
    :parameters (?from - location ?to - location)
    :precondition (and (rob_at ?from) (rob_outside ?from) (door_closed ?from) (loc_connected ?from ?to) )
    :effect ( and (rob_at ?to) (rob_outside ?to) (not (rob_at ?from)) (not (rob_outside ?from)) )
  )

    ;Robot close door 
    (:action rob_close_door
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (door_open ?l) ) 
        :effect (and(not (door_open ?l)) (door_closed ?l))
    
    )

    ;Robot open door 
    (:action rob_open_door
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (door_closed ?l) ) 
        :effect (and(not (door_closed ?l)) (door_open ?l))
    
    )

    ;Move outside location
    (:action rob_move_outside
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (door_open ?l) (rob_inside ?l) ) 
        :effect (and (rob_outside ?l) (not(rob_inside ?l)) )

    )

    ;Move inside location
    (:action rob_move_inside
        :parameters (?l - location)
        :precondition (and (rob_at ?l) (door_open ?l) (rob_outside ?l) ) 
        :effect (and (not(rob_outside ?l)) (rob_inside ?l) )
    )


    ;Robot action to take object in a instance
  (:action rob_take
      :parameters (?o - object ?l - location )
      :precondition (and (rob_at ?l) (rob_inside ?l) (obj_at ?o ?l)) 
      :effect (and (rob_carry ?o) (not (obj_at ?o ?l))) 
  )

    ;Robot action to drop object in a instance
  (:action rob_drop
      :parameters (?o - object ?l - location )
      :precondition (and (rob_at ?l) (rob_inside ?l) (rob_carry ?o))
      :effect (and (not (rob_carry ?o)) (obj_at ?o ?l)) 
  )

  ;Robot action carry object to a person
  (:action robObjToPerson
    :parameters (?o - object ?p - person ?l - location)
    :precondition (and (rob_carry ?o) (rob_at ?l) (rob_inside ?l) (person_at ?p ?l))
    :effect (and (not (rob_carry ?o)) (person_carry ?p ?o)) 
  )

    ;Action person gives object to robot
    (:action personObjToRob
    :parameters (?o - object ?p - person ?l -location )
    :precondition (and (rob_carry ?o) (rob_at ?l) (rob_inside ?l) (person_at ?p ?l))
    :effect (and (rob_carry ?o) (not (person_carry ?p ?o ))) 
  )


)
