from struct import pack, unpack
from .opb import (
    OPB_CONTROLLER,
    OPB_DATA_FMT,
    inc_mmcm_phase,
    set_io_delay,
    )
from .spi import (
    get_spi_control,
    set_spi_control,
    set_spi_register,
    )
import numpy as np
import logging
import sys

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
sh = logging.StreamHandler(stream=sys.stdout)
sh.setLevel(logging.DEBUG)
sh.setFormatter(logging.Formatter('%(asctime)s - %(name)20s - %(levelname)s - %(message)s'))
logger.addHandler(sh)
logger.info('adc5g imported')

def total_glitches(core, bitwidth=8):
    ramp_max = 2**bitwidth - 1
    glitches = 0
    for i in range(len(core)-1):
        diff = core[i+1] - core[i]
        if (diff!=1) and (diff!=-ramp_max):
            glitches += 1
    return glitches


def get_snapshot(roach, snap_name, bitwidth=8, man_trig=True, wait_period=2):
    """
    Reads a one-channel snapshot off the given 
    ROACH and returns the time-ordered samples.
    """

    grab, timestamp = roach.snapshots[snap_name].read_raw(man_trig=man_trig, man_valid=True)
    data = unpack('%ib' %grab['length'], grab['data'])

    return list(d for d in data)


def get_test_vector(roach, snap_names, bitwidth=8, man_trig=True, wait_period=2):
    """
    Sets the ADC to output a test ramp and reads off the ramp,
    one per core. This should allow a calibration of the MMCM
    phase parameter to reduce bit errors.

    core_a, core_c, core_b, core_d = get_test_vector(roach, snap_names)

    NOTE: This function requires the ADC to be in "test" mode, please use 
    set_spi_control(roach, zdok_n, test=1) before-hand to be in the correct 
    mode.
    """
    data_out = []
    cores_per_snap = 4//len(snap_names)
    for snap in snap_names:
        data = get_snapshot(roach, snap, bitwidth, man_trig=man_trig, wait_period=wait_period)
        data_bin = list(((p+128)>>1) ^ (p+128) for p in data)
        for i in range(cores_per_snap):
            data_out.append(data_bin[i::cores_per_snap])
    return data_out


def set_test_mode(roach, zdok_n,counter=True):
    if counter:
        use_counter_test(roach, zdok_n)
    else:
        use_strobe_test(roach, zdok_n)
    if zdok_n == 1:
        logger.warning("DIRTY HACK ALERT: Reading control reg from ZDOK 0 to set ZDOK 1")
    control = get_spi_control(roach, 0)#zdok_n) #sometimes the zdok 1 read interface doesn't work (really worth figuring out why)
    control['test'] = 1
    set_spi_control(roach, zdok_n, **control)


def unset_test_mode(roach, zdok_n):
    if zdok_n == 1:
        logger.warning("DIRTY HACK ALERT: Reading control reg from ZDOK 0 to set ZDOK 1")
    control = get_spi_control(roach, 0)#zdok_n) #sometimes the zdok 1 read interface doesn't work (really worth figuring out why)
    control['test'] = 0
    set_spi_control(roach, zdok_n, **control)


def sync_adc(roach, zdok_0=True, zdok_1=True):
    """
    This sends an external SYNC pulse to the ADC. Set either zdok_0 or 
    zdok_1 to False to not sync those boards

    This should be used after setting test mode on.
    """
    roach.blindwrite(OPB_CONTROLLER, pack('>BBBB', 0x00, 0x00, 0x00, 0x0))
    roach.blindwrite(OPB_CONTROLLER, pack('>BBBB', 0x00, 0x00, 0x00, zdok_0 + zdok_1*2))
    roach.blindwrite(OPB_CONTROLLER, pack('>BBBB', 0x00, 0x00, 0x00, 0x00))

def get_core_offsets(r,snap_names=['snapshot_adc0'],cores=4):
    set_spi_register(r,0,0x05+0x80,0) #use counter
    set_test_mode(r, 0)
    sync_adc(r,zdok_0=True,zdok_1=True)
    test_vec = np.array(get_test_vector(r, snap_names))
    s = test_vec[:,0]
    if np.any(s==255): # Lazy way to make sure we aren't looking at a wrapping section of the counter
        s = test_vec[100]
    offset = np.min(s) - s #these are the relative arrival times. i.e. -1 means arrival is one clock too soon
    return offset



def check_for_glitches(roach, zdok_n, snap_names, bitwidth=8, man_trig=True, wait_period=2, ps_range=56):
    set_test_mode(roach, zdok_n, counter=True)
    sync_adc(roach)
    core_a, core_c, core_b, core_d = get_test_vector(roach, snap_names, man_trig=man_trig, wait_period=wait_period)
    glitches = total_glitches(core_a, 8) + total_glitches(core_c, 8) + \
        total_glitches(core_b, 8) + total_glitches(core_d, 8)
    unset_test_mode(roach, zdok_n)
    sync_adc(roach)
    return glitches
 
def calibrate_mmcm_phase(roach, zdok_n, snap_names, bitwidth=8, man_trig=True, wait_period=2, ps_range=56):
    """
    This function steps through all 56 steps of the MMCM clk-to-out 
    phase and finds total number of glitchss in the test vector ramp 
    per core. It then finds the least glitchy phase step and sets it.
    """
    set_test_mode(roach, zdok_n, counter=True)
    sync_adc(roach)
    glitches_per_ps = []
    # shift until we get to some bad data
    for ps in range(ps_range):
        core_a, core_c, core_b, core_d = get_test_vector(roach, snap_names, man_trig=man_trig, wait_period=wait_period)
        glitches = total_glitches(core_a, 8) + total_glitches(core_c, 8) + \
            total_glitches(core_b, 8) + total_glitches(core_d, 8)
        if glitches > 0:
            break
        inc_mmcm_phase(roach, zdok_n)

    for ps in range(ps_range):
        core_a, core_c, core_b, core_d = get_test_vector(roach, snap_names, man_trig=man_trig, wait_period=wait_period)
        glitches = total_glitches(core_a, 8) + total_glitches(core_c, 8) + \
            total_glitches(core_b, 8) + total_glitches(core_d, 8)
        glitches_per_ps.append(glitches)
        inc_mmcm_phase(roach, zdok_n)
    unset_test_mode(roach, zdok_n)
    zero_glitches = [gl==0 for gl in glitches_per_ps]
    n_zero = 0
    longest_min = 0
    while True:
        try:
            rising  = zero_glitches.index(True, n_zero)
        except ValueError:
            break
        n_zero  = rising + 1
        try:
            falling = zero_glitches.index(False, n_zero)
        except ValueError:
            falling = len(zero_glitches) - 1
        n_zero  = falling + 1
        min_len = falling - rising
        if min_len > longest_min:
            longest_min = min_len
            optimal_ps = rising + int((falling-rising)//2)
    if longest_min==0:
        #raise ValueError("No optimal MMCM phase found!")
        return None, glitches_per_ps
    else:
        # decrement MMCM back to optimal place
        for ps in range(ps_range - optimal_ps):
            inc_mmcm_phase(roach, zdok_n, inc=0)
        return optimal_ps, glitches_per_ps


def get_histogram(roach, zdok_n, core, fmt="hist_{zdok_n}_count_{core}", size=256): 
    """
    Reads histogram data from a Shared BRAM.

    Obviously you must have the histogram block instantiated in your design 
    and the FPGA programmed for this to work. If you have renamed the histogram 
    blocks then edit the 'fmt' paramater. If you've changed the histogram size 
    then change the 'size' parameter.
    """
    counts = unpack('>{}Q'.format(size), roach.read(fmt.format(zdok_n=zdok_n, core=core), size*8))
    return counts[size//2:] + counts[:size//2]

def find_best_delay(d,clk=625.,ref_clk=200.,verbose=False,offset=0,tolerance=None,reference=None):
    '''
    A method to set the best bitwise delays for an input data bus. We assume that all the inputs
    are approximately aligned to begin with. I.e. if the input bus is 8 bits, we assume that the relative delays
    of each bit are << 1 clock cycle.
    Arguments:
        d        : An [N_BITS x N_DELAY_TRIALS] array containing numbers of glitches per bit per delay trial.
        clk      : The data transfer clock in any unit (we assume the data rate is DDR and thus double the clk rate)
        ref_clk  : The FPGA IODELAY reference clock, in the same units as clk.
        verbose  : Boolean value. Set to true to print information about what's going on
        offset   : Set to an integer value to skip <offset> stable eyes. This can be used to synchronize multiple interfaces
                   which are whole cycles offset.
        tolerance: Tolerance sets (indirectly) the minimum delay which we consider to be a valid place to look for an eye.
                   This is to accomodate variation of delays of different bits. I.e., if the start of the eye of bit 0 is found at tap 1,
                   the start of the eye of bit 1 may occur at a lower delay than the IODELAY block can provide. The value of <tolerance> should
                   reflect the maximum variation of delays you expect, in units of IODELAY taps. By default it is assumed that
                   all bits are grouped within half an eye (i.e. clk/4 for DDR). If you have trouble calibrating because the delay range
                   is being exhausted (particularly if you are using non-zero offset), you can try and reduce this value to start
                   searching for an eye closer to the minimum tap delay.


        1. Find the first non-zero value more than <tolerance> taps from the start
        2. Find the first zero following this. This marks the start of the eye we want to capture on
        3. Find the next non-zero value. This marks the end of the capture eye.
        4. Set the delay to mid way between these points. Where the midway is not an integer, use the relative number of glitches
           on each side of the eye to determine the most favourable position.
        5. Repeat for the next bit, but begin searching for the first non-zero value one clock cycle earlier than the eye centre
           we have already found.
    '''
    n_bits, n_taps = d.shape
    tap_delay = 1./ref_clk/float(n_taps)/2. #78ps for 200 MHz reference
    logger.debug("tap_delay: %.1f ps"%(tap_delay*1e6))
    taps_per_cycle = (1./clk)/tap_delay/2. #This gives the number of taps in a complete clock cycle (1/2 because data is DDR)
    logger.debug("taps_per_cycle: %.1f"%taps_per_cycle)
    if reference is None:
        if tolerance is None:
            search_start_point = int(taps_per_cycle*(offset + 0.5))
        else:
            search_start_point = int(taps_per_cycle*offset + tolerance)
    else:
        search_start_point = reference - int(taps_per_cycle)
    # don't start at negative!
    search_start_point = max(0,search_start_point)
    eye_centres = np.zeros(n_bits,dtype=int)
    for bit in range(n_bits):
        logger.debug("Starting search for bit %d eye at tap %d"%(bit,search_start_point))
        for delay in range(search_start_point,n_taps):
            if d[bit,delay] != 0:
                #we have found the glitchy area we were looking for
                #this is where we will start our search for the start of the eye
                first_glitch = delay
                logger.debug("found first glitch at %d"%first_glitch)
                break
            if (d[bit,delay] == 0) and (delay==(n_taps-1)):
                raise Exception("  Couldn't find first glitch")
        for delay in range(first_glitch,n_taps):
            if np.all(d[bit,delay:delay+3] == 0): #Check for runs of 3 zeros (sometimes even outside the eye there will be a delay with no glitches)
                #we have found the start of the eye
                eye_start = delay
                logger.debug("  found eye start at %d"%eye_start)
                #record the glitches one tap earlier to help decide which of the
                #two best taps to use if the number of "good" delays is even
                glitches_before_eye = d[bit,delay-1]
                logger.debug("    glitches before eye: %d"%glitches_before_eye)
                break
            if (d[bit,delay] != 0) and (delay==(n_taps-4)):
                logger.critical("Couldn't find start of eye!")
                raise Exception("Couldn't find start of eye")
        for delay in range(eye_start,n_taps):
            if d[bit,delay] != 0:
                #we have found the end of the eye
                eye_end = delay-1
                logger.debug("  found eye end at %d"%eye_end)
                #record the glitches one tap after the eye closes to help decide which of the
                #two best taps to use if the number of "good" delays is even
                glitches_after_eye = d[bit,delay]
                logger.debug("    glitches after eye: %d"%glitches_after_eye)
                break
            if (d[bit,delay] == 0) and (delay==(n_taps-1)):
                logger.debug("Couldn't find end of eye, choosing half cycle from sys start")
                eye_end = int(eye_start + taps_per_cycle)
                glitches_after_eye = 0
        # Find the middle of the eye
        eye_centre = eye_start + (eye_end - eye_start)/2.
        logger.debug("  EYE CENTRE at %.1f"%eye_centre)
        # tie break the non integer case
        if eye_centre % 1 != 0:
            if glitches_after_eye >= glitches_before_eye:
                eye_centre = np.floor(eye_centre)
            else:
                eye_centre = np.ceil(eye_centre)
            logger.debug("    TIEBREAK: EYE CENTRE at %d"%eye_centre)
        eye_centres[bit] = int(eye_centre)
        if bit == 0:
            #If this is the first bit, use it to define the reference point about which we search for
            #the eyes of other bits
            search_start_point = eye_centres[0] - int(taps_per_cycle)
            if search_start_point < 0:
                search_start_point = 0
            logger.debug("  NEW START SEARCH REFERENCE POINT IS %d"%search_start_point)
    return eye_centres

def count_bit_glitches(d,bit):
    """
    With data grabbed from the adc in strobing test mode, pass an [NBITS x NDELAYS] array
    and compute (in a rough and ready not really properly counting kind of way)
    how many glitches there are for each bit
    """
    SPACING = 11
    glitches = 0
    data = (np.array(d) & (1<<bit))>>bit
    last_index = None
    for i in range(len(d)-SPACING):
        if i%SPACING==0:
            #print "bit %d"%bit, data[i:i+SPACING]
            if data[i:i+SPACING].sum() != 1:
                glitches += 1
        if data[i]==1:
            if (last_index is not None) and ((i-last_index) != SPACING):
                glitches += 1
            last_index = i
    return glitches

def use_strobe_test(r,zdok):
    set_spi_register(r,zdok,0x05+0x80,1)

def use_counter_test(r,zdok):
    set_spi_register(r,zdok,0x05+0x80,0)

def get_glitches_per_bit(r,zdok,snaps=['snapshot_adc0'],delays=32,bits=8,cores=4,verbose=False):
    set_test_mode(r,zdok,counter=False)
    sync_adc(r)
    glitches = np.zeros([cores,bits,delays],dtype=int)
    for delay in range(delays):
        logger.debug("setting delay %d"%(delay))
        for core in range(cores):
            set_io_delay(r,zdok,core,delay)
        test_vec = np.array(get_test_vector(r,snaps))
        for core in range(cores):
            for bit in range(bits):
                glitches[core,bit,delay] = count_bit_glitches(test_vec[core],bit)
    if verbose:
        for core in range(cores):
            logger.debug("##### GLITCHES FOR CORE %d BY IODELAY #####"%core)
            for delay in range(delays):
                string = ''
                string += "%2d:"%delay
                for bit in range(bits):
                    string += "%4d"%(glitches[core,bit,delay])
                string += "TOTAL %d"%(glitches.sum(axis=1)[core,delay])
                logger.debug(string)
    unset_test_mode(r,zdok)
    return glitches

def calibrate_all_delays(r,zdok,snaps=['snapshot_adc0'],verbosity=1):
    """
    Put an ADC in stobe test mode, find the glitches per bit per delay,
    find the optimum delays, and load them
    """
    glitches = get_glitches_per_bit(r,zdok,snaps=snaps,verbose=(verbosity>1))
    cores, bits, taps = glitches.shape
    best_delay = np.zeros([cores,bits], dtype=int)
    # get the delay for core 0, and then use this as a reference for the other cores. This (should) prevent
    # cores getting a clock cycle out of sync
    best_delay[0] = find_best_delay(glitches[0],verbose=(verbosity>2),reference=None)
    for core in range(1,cores):
        best_delay[core] = find_best_delay(glitches[core],verbose=(verbosity>2),reference=best_delay[0,0])
    set_adc_iodelays(r,zdok,best_delay,verbose=(verbosity>0))
    return best_delay

def set_adc_iodelays(r,zdok,delays,verbose=False):
    """
    Pass an NCORES x NDELAYS array to write the delays of the IODELAY blocks
    """
    cores, bits = delays.shape
    for core in range(cores):
        logger.info("setting adc %d core %d delays: %s"%(zdok, core, str(delays[core])))
        for bit in range(bits):
            set_io_delay(r,zdok,core,delays[core,bit],bit=bit)
