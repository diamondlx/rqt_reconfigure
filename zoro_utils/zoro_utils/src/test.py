import _zoro_utils as z

#z.zoro_init(logfile="logfile.log", logconfig="log.cfg")
z.zoro_init(logprefix="abc", logconfig="log.cfg")

for i in range(100000000):
    s = str(i)
    z.get_logger("nnnn").info(s+":info")
    z.get_logger("nnnn").debug(s+":debug")
    z.get_logger("nnnn").warn(s+":warn")
    z.get_logger("nnnn").error(s+":error")
    z.get_logger("nnnn").fatal(s+":fatal")

exit(0)

z.debug("debug")
z.info("info")
z.warn("warn")
z.error("error")
z.fatal("fatal")
