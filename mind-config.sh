## ----------------------- mind Configuration File Override ----------------------------

##
## HINT: You can copy the contents of this file to a mind-config.local.sh 
##       file that is untrackable by git and readable by mind.
##

## ----------------------- mind Configuration File Override ----------------------------
## ACTIVE Modules CONFIGURATION
## List of active modules to run, defined in docker-compose.yaml.
##
## Possible values:
##   - vis_tools        :   starts tools for data visualization (foxglove)
##   - stewart          :   starts stewart platform

ACTIVE_MODULES="stewart"

################################# MODE OF OPERATION #################################
## Possible modes of operation when running mind.
## Possible values:
##	 - deploy (default)		:	runs production-grade containers (non-editable)
##	 - develop   		    :	runs developer containers (editable)

MODE_OF_OPERATION="deploy"

############################## ADVANCED CONFIGURATIONS ##############################
## Name to append to docker containers. DEFAULT = "<your_watcloud_username>"
COMPOSE_PROJECT_NAME="bishneet"

## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = "<your_current_github_branch>"
TAG="bishneet"

# Docker Registry to pull/push images. DEFAULT = "ghcr.io/watonomous/wato_monorepo"
# REGISTRY_URL=""