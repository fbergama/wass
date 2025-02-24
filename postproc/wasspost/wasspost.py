import argparse
from argparse import RawDescriptionHelpFormatter
from netCDF4 import Dataset
import numpy as np

VERSION="0.0.1"



def action_info( ncfile ):
    print(f"Opening {ncfile}")
    S = ""
    with Dataset( ncfile, "r") as ds:
        S = ""
        S += "\nVariables:\n\n"
        for v in ds.variables:
            S += "%s - %s (%s)\n"%(v, ds.variables[v].shape,ds.variables[v].dtype)
        #S += "\nAttributes:\n\n"
        #for a in ds.ncattrs():
        #    S += " - %s = %s\n"%(a,ds.getncattr(a))
        S += "\nGroups: \n"
        for g in ds.groups:
            S += " %s\n"%ds[g].path
            S += "\n  Variables:\n"
            for v in ds[g].variables:
                S += "    - %s: \n%s\n"%(v,np.array(ds[g].variables[v]))
            S += "\n  ncattrs:\n"
            for a in ds[g].ncattrs():
                S += "    %s = %s\n"%(a,ds[g].getncattr(a))

        print(S)


def get_action_description():
    return """
    Post-processing operations:
    ----------------
        info: prints some info about the specified nc file
        visibilitymap: compute visibility map for each grid point 
        texture: generate surface grid texture
        """

def wasspost_main():
    parser = argparse.ArgumentParser(
                        prog='wasspost',
                        description='WASS NetCDF post processing tool',
                        epilog=get_action_description(),
                        formatter_class=RawDescriptionHelpFormatter )
    parser.add_argument('action', choices=['info','visibilitymap','texture'], help='post-processing operation to perform (see below)')
    parser.add_argument('ncfile', help='The NetCDF file to post-process, produced by WASS or WASSfast')
    args = parser.parse_args()


    if args.action=="info":
        action_info( args.ncfile )
