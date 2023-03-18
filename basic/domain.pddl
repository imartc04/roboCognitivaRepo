(define (domain robot-navigation)
  (:requirements :typing :strips :disjunctive-preconditions )
  (:types location object person)
  (:predicates 
    (obj_at ?o - object ?l - location)
    (person_at ?p - person ?l - location)
    (rob_at ?l - location)
    (loc_connected ?l1 - location ?l2 - location)
    (rob_carry ?o - object)
    (person_carry ?p - person ?o - object)

  )

  ;Robot moves between locations
  (:action rob_move
    :parameters (?from - location ?to - location)
    :precondition (and (rob_at ?from) (loc_connected ?from ?to) )
    :effect ( and (rob_at ?to) (not (rob_at ?from)))
  )

    ;Robot action to take object in a instance
  (:action rob_take
      :parameters (?o - object ?ol - location )
      :precondition (and (rob_at ?ol) (obj_at ?o ?ol)) 
      :effect (and (rob_carry ?o) (not (obj_at ?o ?ol))) 
  )

    ;Robot action to drop object in a instance
  (:action rob_drop
      :parameters (?o - object ?dl - location )
      :precondition (and (rob_at ?dl) (rob_carry ?o))
      :effect (and (not (rob_carry ?o)) (obj_at ?o ?dl)) 
  )
  
  ;Robot action carry object to a person
  (:action robObjToPerson
    :parameters (?o - object ?p - person ?rl - location)
    :precondition (and (rob_carry ?o) (rob_at ?rl) (person_at ?p ?rl))
    :effect (and (not (rob_carry ?o)) (person_carry ?p ?o)) 
  )

    ;Action person gives object to robot
    (:action personObjToRob
    :parameters (?o - object ?p - person ?rl -location )
    :precondition (and (rob_carry ?o) (rob_at ?rl) (person_at ?p ?rl))
    :effect (and (rob_carry ?o) (not (person_carry ?p ?o ))) 
  )


)
