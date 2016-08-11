# DocPad Configuration File
# http://docpad.org/docs/config

# Define the DocPad Configuration
docpadConfig = {
	# ...
	#
# Out Path
# Where should we put our generated website files?
# If it is a relative path, it will have the resolved `rootPath` prepended to it
outPath: 'out/wass'  # default

templateData:  # example

        # Specify some site properties
        site:
            # The production URL of our website
            url: "http://www.dais.unive.it/wass"

            # The default title of our website
            title: "WASS project"

            # The website description (for SEO)
            description: "WASS project homepage"

            # The website keywords (for SEO) separated by commas
            keywords: ""

}

# Export the DocPad Configuration
module.exports = docpadConfig
