#ifndef ERR_H_
#define ERR_H_

#define E_OK (0)            /* 0 indicates no error */
#define E_INVALID_CMD (-1)  /* Invalid command code */
#define E_INVALID_ARG (-2)  /* Invalid argument to command */
#define E_WRONG_ARGC (-3)   /* Wrong number of arguments */
#define E_PKT_TOO_LONG (-4) /* Packet too long */
#define E_SPI (-5)          /* SPI related error */
#define E_LINK_LOSS_RESET (-6) /* System is resetting due to loss of link */
#define E_CMD_DROPPED (-7)  /* A UART command in the buffer was dropped due to incoming remote CMD */
#define E_TIMEOUT (-8)	/* Operation timeout for unspecified reason */
#endif
