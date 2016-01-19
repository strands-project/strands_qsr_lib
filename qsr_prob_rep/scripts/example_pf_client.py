#!/usr/bin/env python
# -*- coding: utf-8 -*-

from qsrrep_ros.ros_client import ROSClient
from qsrrep_lib.rep_io_pf import PfRepRequestCreate,PfRepRequestPredict, PfRepRequestUpdate, PfRepRequestList, PfRepRequestRemove
from qsrrep_pf.pf_model import PfModel
import os
import rospy
import json
import numpy as np
import argparse

def load_lookup(filename):
    with open(filename, 'r') as f:
        return json.load(f)

def load_files(path):
    m = PfModel()
    for f in os.listdir(path):
        if f.endswith(".pred"):
            name = f.split('.')[0]
            filename = path + '/' + f
            with open(filename, 'r') as a:
                m.add_prediction_matrix(name, np.loadtxt(a))

        elif f.endswith(".obs"):
            name = f.split('.')[0]
            filename = path + '/' + f
            with open(filename, 'r') as a:
                m.add_observation_matrix(name, np.loadtxt(a))

    return m.get()


if __name__ == "__main__":

    # General parsers
    general = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,add_help=False)
    parser = argparse.ArgumentParser(parents=[general])
    subparsers = parser.add_subparsers(dest='action')
    debug_parse = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,add_help=False)
    debug_parse.add_argument('--debug', help="Prints debug information", action="store_true", required=False, default=False)

    # Parsers for create function
    create_parse = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,add_help=False)
    create_parse.add_argument('-m', '--model', help="reads *.pred and *.obs files from the given directory to create models. Files have to have the same name to be part of the same model.", type=str, required=True)
    create_parse.add_argument('-s', '--states', help="reads the given file as a state lookup table.", type=str, required=True)
    create_parse.add_argument('-n', '--num_particles', help="the number of particles.", type=int, required=True)
    create_parse.add_argument('--ensure_particle_per_state', help="if set, each state is represented by at least one particle before they are drawn uniformly.", action="store_true", default=False)
    create_parse.add_argument('--starvation_factor', help="[0,1] percentage of particle drawn by random to prevent starvation during update. Default 0.1.", type=float, required=False, default=0.1)
    create_parse.add_argument('--uuid', help="the uuid of the particle filter to use. If not set, a new one will be created", type=str, required=False, default="")
    subparsers.add_parser('create',parents=[general, debug_parse, create_parse])

    # Parsers for sample function
    predict_parse = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,add_help=False)
    predict_parse.add_argument('-u', '--uuid', help="the uuid of the particle filter to use", type=str, required=True)
    predict_parse.add_argument('-n', '--num_steps', help="the number of sample generations", type=int, required=True)
    subparsers.add_parser('predict', parents=[general, predict_parse, debug_parse])

    # Parsers for loglikelihood function
    up_parse = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,add_help=False)
    up_parse.add_argument('-u', '--uuid', help="the uuid of the particle filter to use", type=str, required=True)
    up_parse.add_argument('-o', '--obs', help="the state currently observed", type=str, required=True)
    subparsers.add_parser('update', parents=[general, up_parse, debug_parse])

    list_parse = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,add_help=False)
    subparsers.add_parser('list', parents=[general, list_parse])

    remove_parse = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,add_help=False)
    remove_parse.add_argument('-u', '--uuid', help="the uuid of the particle filter to use", type=str, required=True)
    subparsers.add_parser('remove', parents=[general, remove_parse])

    # Parse arguments
    args = parser.parse_args()

    rospy.init_node("ros_client")
    r = ROSClient()

    if args.action == "create":
        models = load_files(args.model)
        states = load_lookup(args.states)
        d = r.call_service(
            PfRepRequestCreate(
                num_particles=args.num_particles,
                models=models,
                state_lookup_table=states,
                starvation_factor=args.starvation_factor,
                ensure_particle_per_state=args.ensure_particle_per_state,
                debug=args.debug,
                uuid=args.uuid if args.uuid != "" else None
            )
        )
        print d

    elif args.action == "predict":
        p = r.call_service(
            PfRepRequestPredict(
                uuid=args.uuid,
                num_steps=args.num_steps,
                debug=args.debug
            )
        )
        print p
    elif args.action == "update":
        p = r.call_service(
            PfRepRequestUpdate(
                uuid=args.uuid,
                observation=args.obs,
                debug=args.debug
            )
        )
        print p
    elif args.action == "list":
        p = r.call_service(
            PfRepRequestList()
        )
        print p
    elif args.action == "remove":
        p = r.call_service(
            PfRepRequestRemove(
                uuid=args.uuid
            )
        )
        print p
