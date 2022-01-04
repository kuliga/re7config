library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.PCK_CRC32_D32.ALL;

entity crc32blaze_v1_0_S00_AXIS is
	generic (
		-- Users to add parameters here

		-- User parameters ends
		-- Do not modify the parameters beyond this line

		-- AXI4Stream sink: Data Width
		C_S_AXIS_TDATA_WIDTH	: integer	:= 32
	);
	port (
		-- Users to add ports here
        block_size  : in std_logic_vector(31 downto 0);
        init    : in std_logic;
        checksum_done   : out std_logic;
        checksum    : out std_logic_vector(31 downto 0);
        
		-- User ports ends
		-- Do not modify the ports beyond this line

		-- AXI4Stream sink: Clock
		S_AXIS_ACLK	: in std_logic;
		-- AXI4Stream sink: Reset
		S_AXIS_ARESETN	: in std_logic;
		-- Ready to accept data in
		S_AXIS_TREADY	: out std_logic;
		-- Data in
		S_AXIS_TDATA	: in std_logic_vector(C_S_AXIS_TDATA_WIDTH-1 downto 0);
		-- Byte qualifier
		S_AXIS_TSTRB	: in std_logic_vector((C_S_AXIS_TDATA_WIDTH/8)-1 downto 0);
		-- Indicates boundary of last packet
		S_AXIS_TLAST	: in std_logic;
		-- Data is in valid
		S_AXIS_TVALID	: in std_logic
	);
end crc32blaze_v1_0_S00_AXIS;

architecture arch_imp of crc32blaze_v1_0_S00_AXIS is
	-- https://groups.google.com/d/msg/comp.lang.vhdl/eBZQXrw2Ngk/4H7oL8hdHMcJ
	function reverse_any_vector (a: in std_logic_vector) return std_logic_vector is
	   variable result: std_logic_vector(a'RANGE);
	   alias aa: std_logic_vector(a'REVERSE_RANGE) is a;
	begin
	   for i in aa'RANGE loop
		  result(i) := aa(i);
	   end loop;
	   return result;
	end; -- function reverse_any_vector

	signal axis_tready	: std_logic;
    signal tmp_crc      : std_logic_vector(31 downto 0);
    signal wr_ena       : std_logic;
    signal s_checksum_done  : std_logic;
    
begin

	S_AXIS_TREADY	<= axis_tready;
	wr_ena <= axis_tready and S_AXIS_TVALID;
	checksum_done <= s_checksum_done;
	
	process(S_AXIS_ACLK)
	begin
	  if (rising_edge (S_AXIS_ACLK)) then
	    if(S_AXIS_ARESETN = '0' or init = '1') then
            axis_tready <= '1'; -- crc32 block ready for the first data beat
            tmp_crc <= (others => '1'); --initiliaze crc32 block with 0xFFFFFFFF
            checksum <= x"DEADBEEF"; -- make it easy to debug
            s_checksum_done <= '0'; 
	    else
	        if (s_checksum_done = '1') then
	           checksum <= not reverse_any_vector(tmp_crc);
            else
                if (wr_ena = '1') then   
                    tmp_crc <= nextCRC32_D32(reverse_any_vector(S_AXIS_TDATA), tmp_crc);
                    if (S_AXIS_TLAST = '1') then
                        s_checksum_done <= '1';
                        axis_tready <= '0';
                    else
                        s_checksum_done <= '0';
                        axis_tready <= '1'; -- crc32 block does calculation in one clock cycle
                    end if;
                end if;
            end if;
	    end if;  
	  end if;
	end process;


	-- Add user logic here

	-- User logic ends

end arch_imp;
