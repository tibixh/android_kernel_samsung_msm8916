/*
 * =================================================================
 *
 *
 *	Description:  samsung display panel file
 *
 *	Author: jb09.kim
 *	Company:  Samsung Electronics
 *
 * ================================================================
 */
/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) 2012, Samsung Electronics. All rights reserved.

*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#include "ss_dsi_panel_S6E88A0_SS_WVGA.h"
#include "ss_dsi_mdnie_S6E88A0_SS_WVGA.h"

/*
 * The panel only answers the B5/C8 NV reads once it has been woken up
 * (sleep-out). The commonized on_pre flow runs the reads BEFORE the
 * panel-on command sequence, and unlike the stock boot chain there is
 * no continuous splash, so the
 * panel is still asleep at that point and every NV read returns zero.
 * Defer all NV reads until panel on post (after the on commands).
 */
static bool panel_woken;

static int mdss_manufacture_date_read(struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_mdnie_read(struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_samart_dimming_init(struct mdss_dsi_ctrl_pdata *ctrl);

/*
 * The heat panel only answers NV register reads when the DSI command
 * engine runs in low power mode. The stock driver toggled the tx power
 * mode around every panel read, so do the same here.
 */
extern void mdss_dsi_set_tx_power_mode(int mode, struct mdss_panel_data *pdata);

/*
 * Panel register read helper.
 *
 * mdss_samsung_panel_data_read() can not be used here: the command send
 * helper it relies on (mdss_samsung_send_cmd) silently drops every
 * command while vdd_blank_mode is still FB_BLANK_POWERDOWN, which is
 * the state during the first panel-on when these reads run. As a result
 * the level keys, packet size and B0 read position commands would never
 * reach the panel and only bare-readable registers (DA/DB/DC) answer.
 * Send the sequence directly like the stock driver did instead.
 *
 * Level keys and tx power mode are managed by the caller, since the
 * panel only answers NV reads inside one single key session (the stock
 * driver read MTP, mdnie and date all between one key enable/disable).
 */
static int mdss_panel_data_read_raw(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *rx_cmds, char *buffer)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);
	struct dsi_panel_cmds *pcmds;
	int ndx, packet_size, read_size, read_pos, startoffset;
	int loop_limit, read_count, total = 0;
	int j;

	if (IS_ERR_OR_NULL(vdd) || IS_ERR_OR_NULL(rx_cmds) ||
			IS_ERR_OR_NULL(rx_cmds->cmds))
		return -EINVAL;

	ndx = display_ndx_check(ctrl);

	pcmds = &vdd->dtsi_data[ndx].packet_size_tx_cmds[vdd->panel_revision];
	packet_size = pcmds->cmds->payload[0];
	mdss_dsi_panel_cmds_send(ctrl, pcmds);

	read_size = rx_cmds->read_size[0];
	startoffset = read_pos = rx_cmds->read_startoffset[0];

	loop_limit = (read_size + packet_size - 1) / packet_size;
	for (j = 0; j < loop_limit; j++) {
		pcmds = &vdd->dtsi_data[ndx].reg_read_pos_tx_cmds[vdd->panel_revision];
		pcmds->cmds->payload[1] = read_pos;
		mdss_dsi_panel_cmds_send(ctrl, pcmds);

		read_count = mdss_samsung_panel_cmd_read(ctrl, rx_cmds,
			((read_size - read_pos + startoffset) < packet_size) ?
			(read_size - read_pos + startoffset) : packet_size);

		if (read_count > 0 && (total + read_count) <= read_size) {
			memcpy(buffer + total, ctrl->rx_buf.data, read_count);
			total += read_count;
			read_pos += read_count;
		}

		if (read_pos - startoffset >= read_size)
			break;
	}

	pr_info("%s: cmd 0x%02x size %d got %d bytes", __func__,
		rx_cmds->cmds->payload[0], read_size, total);

	return total;
}

static int mdss_panel_on_pre(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return false;
	}

	pr_info("%s %d\n", __func__, ctrl->ndx);

	mdss_panel_attach_set(ctrl, true);

	return true;
}

static int mdss_panel_on_post(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return false;
	}

	pr_info("%s %d\n", __func__, ctrl->ndx);

	/* The panel is awake now (on commands sent), NV reads can run */
	panel_woken = true;

	/*
	 * Run all NV reads inside one single key session, MTP first,
	 * exactly like the stock driver did: the panel stops answering
	 * NV reads once the test keys have been disabled and re-enabled.
	 */
	mdss_dsi_set_tx_power_mode(1, &ctrl->panel_data);
	mdss_dsi_panel_cmds_send(ctrl,
		&vdd->dtsi_data[ctrl->ndx].level1_key_enable_tx_cmds[vdd->panel_revision]);
	mdss_dsi_panel_cmds_send(ctrl,
		&vdd->dtsi_data[ctrl->ndx].level2_key_enable_tx_cmds[vdd->panel_revision]);

	if (!vdd->smart_dimming_loaded_dsi[ctrl->ndx])
		vdd->smart_dimming_loaded_dsi[ctrl->ndx] =
			mdss_samart_dimming_init(ctrl);

	if (!vdd->mdnie_loaded_dsi[ctrl->ndx])
		vdd->mdnie_loaded_dsi[ctrl->ndx] =
			mdss_mdnie_read(ctrl);

	if (!vdd->manufacture_date_loaded_dsi[ctrl->ndx])
		vdd->manufacture_date_loaded_dsi[ctrl->ndx] =
			mdss_manufacture_date_read(ctrl);

	mdss_dsi_panel_cmds_send(ctrl,
		&vdd->dtsi_data[ctrl->ndx].level2_key_disable_tx_cmds[vdd->panel_revision]);
	mdss_dsi_panel_cmds_send(ctrl,
		&vdd->dtsi_data[ctrl->ndx].level1_key_disable_tx_cmds[vdd->panel_revision]);
	mdss_dsi_set_tx_power_mode(0, &ctrl->panel_data);

	return true;
}

static int mdss_panel_revision(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return false;
	}

	if (vdd->manufacture_id_dsi[ctrl->ndx] == 0)
		mdss_panel_attach_set(ctrl, false);
	else
		mdss_panel_attach_set(ctrl, true);

	vdd->panel_revision = 'A' - 'A';

	return true;
}

static int mdss_manufacture_date_read(struct mdss_dsi_ctrl_pdata *ctrl)
{
	unsigned char date[2];
	int year, month, day;
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return false;
	}

	if (!panel_woken) {
		pr_info("%s: panel not woken yet, defer\n", __func__);
		return false;
	}

	/* Read mtp (B5h 24,25th) for manufacture date */
	if (vdd->dtsi_data[ctrl->ndx].manufacture_date_rx_cmds[vdd->panel_revision].cmd_cnt) {
		mdss_panel_data_read_raw(ctrl,
			&vdd->dtsi_data[ctrl->ndx].manufacture_date_rx_cmds[vdd->panel_revision],
			date);

		year = date[0] >> 4;
		year += 2011; // 0 = 2011 year
		month = date[0] & 0x0f;
		day = date[1];

		vdd->manufacture_date_dsi[ctrl->ndx] = year * 10000 + month * 100 + day;
		vdd->manufacture_time_dsi[ctrl->ndx] = 0;

		pr_err("manufacture_date DSI%d = (%d%04d) - year(%d) month(%d) day(%d)\n",
			ctrl->ndx, vdd->manufacture_date_dsi[ctrl->ndx], vdd->manufacture_time_dsi[ctrl->ndx],
			year, month, day);

	} else {
		pr_err("%s DSI%d error (%d)", __func__, ctrl->ndx, vdd->panel_revision);
		return false;
	}

	return true;
}

#define COORDINATE_DATA_SIZE 6
#define MDNIE_SCR_WR_ADDR 36

#define F1(x,y) ((y)-((164*(x))/151)+8)
#define F2(x,y) ((y)-((70*(x))/67)-7)
#define F3(x,y) ((y)+((181*(x))/35)-18852)
#define F4(x,y) ((y)+((157*(x))/52)-12055)

static char coordinate_data[][COORDINATE_DATA_SIZE] = {
	{0xff, 0x00, 0xff, 0x00, 0xff, 0x00}, /* dummy */
	{0xff, 0x00, 0xfa, 0x00, 0xfa, 0x00}, /* Tune_1 */
	{0xff, 0x00, 0xfb, 0x00, 0xfe, 0x00}, /* Tune_2 */
	{0xfc, 0x00, 0xfb, 0x00, 0xff, 0x00}, /* Tune_3 */
	{0xff, 0x00, 0xfe, 0x00, 0xfb, 0x00}, /* Tune_4 */
	{0xff, 0x00, 0xff, 0x00, 0xff, 0x00}, /* Tune_5 */
	{0xfa, 0x00, 0xfc, 0x00, 0xff, 0x00}, /* Tune_6 */
	{0xfc, 0x00, 0xff, 0x00, 0xf9, 0x00}, /* Tune_7 */
	{0xfb, 0x00, 0xff, 0x00, 0xfb, 0x00}, /* Tune_8 */
	{0xfa, 0x00, 0xff, 0x00, 0xff, 0x00}, /* Tune_9 */
};

static int mdnie_coordinate_index(int x, int y)
{
	int tune_number = 0;

	if (F1(x,y) > 0) {
		if (F3(x,y) > 0) {
			tune_number = 3;
		} else {
			if (F4(x,y) < 0)
				tune_number = 1;
			else
				tune_number = 2;
		}
	} else {
		if (F2(x,y) < 0) {
			if (F3(x,y) > 0) {
				tune_number = 9;
			} else {
				if (F4(x,y) < 0)
					tune_number = 7;
				else
					tune_number = 8;
			}
		} else {
			if (F3(x,y) > 0)
				tune_number = 6;
			else {
				if (F4(x,y) < 0)
					tune_number = 4;
				else
					tune_number = 5;
			}
		}
	}

	return tune_number;
}

static int mdss_mdnie_read(struct mdss_dsi_ctrl_pdata *ctrl)
{
	char x_y_location[4];
	int mdnie_tune_index = 0;
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return false;
	}

	if (!panel_woken) {
		pr_info("%s: panel not woken yet, defer\n", __func__);
		return false;
	}

	if (vdd->dtsi_data[ctrl->ndx].mdnie_read_rx_cmds[vdd->panel_revision].cmd_cnt) {
		mdss_panel_data_read_raw(ctrl,
			&(vdd->dtsi_data[ctrl->ndx].mdnie_read_rx_cmds[vdd->panel_revision]),
			x_y_location);

		vdd->mdnie_x[ctrl->ndx] = x_y_location[0] << 8 | x_y_location[1];	/* X */
		vdd->mdnie_y[ctrl->ndx] = x_y_location[2] << 8 | x_y_location[3];	/* Y */

		pr_info("%s: raw mdnie x/y: %02x %02x %02x %02x\n", __func__,
			(unsigned char)x_y_location[0], (unsigned char)x_y_location[1],
			(unsigned char)x_y_location[2], (unsigned char)x_y_location[3]);

		mdnie_tune_index = mdnie_coordinate_index(vdd->mdnie_x[ctrl->ndx], vdd->mdnie_y[ctrl->ndx]);
		coordinate_tunning(ctrl->ndx, coordinate_data[mdnie_tune_index],
			MDNIE_SCR_WR_ADDR, COORDINATE_DATA_SIZE);

		pr_info("%s DSI%d : X-%d Y-%d \n", __func__, ctrl->ndx,
			vdd->mdnie_x[ctrl->ndx], vdd->mdnie_y[ctrl->ndx]);
	} else {
		pr_err("%s DSI%d error", __func__, ctrl->ndx);
		return false;
	}

	return true;
}

static int mdss_samart_dimming_init(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return false;
	}

	if (!panel_woken) {
		pr_info("%s: panel not woken yet, defer\n", __func__);
		return false;
	}

	vdd->smart_dimming_dsi[ctrl->ndx] = vdd->panel_func.samsung_smart_get_conf();

	if (IS_ERR_OR_NULL(vdd->smart_dimming_dsi[ctrl->ndx])) {
		pr_err("%s DSI%d error", __func__, ctrl->ndx);
		return false;
	} else {
		mdss_panel_data_read_raw(ctrl,
			&(vdd->dtsi_data[ctrl->ndx].smart_dimming_mtp_rx_cmds[vdd->panel_revision]),
			vdd->smart_dimming_dsi[ctrl->ndx]->mtp_buffer);

		/* Initialize smart dimming related things here */
		/* lux_tab setting for 350cd */
		vdd->smart_dimming_dsi[ctrl->ndx]->lux_tab = vdd->dtsi_data[ctrl->ndx].candela_map_table[vdd->panel_revision].lux_tab;
		vdd->smart_dimming_dsi[ctrl->ndx]->lux_tabsize = vdd->dtsi_data[ctrl->ndx].candela_map_table[vdd->panel_revision].lux_tab_size;
		vdd->smart_dimming_dsi[ctrl->ndx]->man_id = vdd->manufacture_id_dsi[ctrl->ndx];

		/* Just a safety check to ensure smart dimming data is initialised well */
		vdd->smart_dimming_dsi[ctrl->ndx]->init(vdd->smart_dimming_dsi[ctrl->ndx]);

		vdd->smart_dimming_loaded_dsi[ctrl->ndx] = true;
	}

	pr_info("%s DSI%d : --\n", __func__, ctrl->ndx);

	return true;
}

static struct dsi_panel_cmds aid_cmd;
static struct dsi_panel_cmds *mdss_aid(struct mdss_dsi_ctrl_pdata *ctrl, int *level_key)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);
	int cd_index = 0;
	int cmd_idx = 0;

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return NULL;
	}

	cd_index = get_cmd_index(vdd, ctrl->ndx);

	if (!vdd->dtsi_data[ctrl->ndx].aid_map_table[vdd->panel_revision].size ||
		cd_index > vdd->dtsi_data[ctrl->ndx].aid_map_table[vdd->panel_revision].size)
		goto end;

	cmd_idx = vdd->dtsi_data[ctrl->ndx].aid_map_table[vdd->panel_revision].cmd_idx[cd_index];

	aid_cmd.cmds = &(vdd->dtsi_data[ctrl->ndx].aid_tx_cmds[vdd->panel_revision].cmds[cmd_idx]);
	aid_cmd.cmd_cnt = 1;

	*level_key = PANEL_LEVE1_KEY;

	return &aid_cmd;

end :
	pr_err("%s error", __func__);
	return NULL;
}

static struct dsi_panel_cmds *mdss_acl_off(struct mdss_dsi_ctrl_pdata *ctrl, int *level_key)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return NULL;
	}

	*level_key = PANEL_LEVE1_KEY;

	return &(vdd->dtsi_data[ctrl->ndx].acl_off_tx_cmds[vdd->panel_revision]);
}

static struct dsi_panel_cmds acl_percent_cmd;
static struct dsi_panel_cmds *mdss_acl_percent(struct mdss_dsi_ctrl_pdata *ctrl, int *level_key)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);
	int cd_index = 0;
	int cmd_idx = 0;

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return NULL;
	}

	cd_index = get_cmd_index(vdd, ctrl->ndx);

	if (!vdd->dtsi_data[ctrl->ndx].acl_map_table[vdd->panel_revision].size ||
		cd_index > vdd->dtsi_data[ctrl->ndx].acl_map_table[vdd->panel_revision].size)
		goto end;

	cmd_idx = vdd->dtsi_data[ctrl->ndx].acl_map_table[vdd->panel_revision].cmd_idx[cd_index];

	acl_percent_cmd.cmds = &(vdd->dtsi_data[ctrl->ndx].acl_percent_tx_cmds[vdd->panel_revision].cmds[cmd_idx]);
	acl_percent_cmd.cmd_cnt = 1;

	*level_key = PANEL_LEVE1_KEY;

	return &acl_percent_cmd;

end :
	pr_err("%s error", __func__);
	return NULL;
}

static struct dsi_panel_cmds elvss_cmd;
static struct dsi_panel_cmds *mdss_elvss(struct mdss_dsi_ctrl_pdata *ctrl, int *level_key)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);
	int cd_index = 0;
	int cmd_idx = 0;

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return NULL;
	}

	cd_index = get_cmd_index(vdd, ctrl->ndx);

	if (!vdd->dtsi_data[ctrl->ndx].smart_acl_elvss_map_table[vdd->panel_revision].size ||
		cd_index > vdd->dtsi_data[ctrl->ndx].smart_acl_elvss_map_table[vdd->panel_revision].size)
		goto end;

	cmd_idx = vdd->dtsi_data[ctrl->ndx].smart_acl_elvss_map_table[vdd->panel_revision].cmd_idx[cd_index];

	/* SMART_ACL : use smart acl elvss list when ACL is enabled */
	if ((vdd->acl_status || vdd->siop_status) &&
		vdd->dtsi_data[ctrl->ndx].smart_acl_elvss_tx_cmds[vdd->panel_revision].cmd_cnt)
		elvss_cmd.cmds = &(vdd->dtsi_data[ctrl->ndx].smart_acl_elvss_tx_cmds[vdd->panel_revision].cmds[cmd_idx]);
	else
		elvss_cmd.cmds = &(vdd->dtsi_data[ctrl->ndx].elvss_tx_cmds[vdd->panel_revision].cmds[cmd_idx]);
	elvss_cmd.cmd_cnt = 1;

	*level_key = PANEL_LEVE1_KEY;

	pr_info("%s : cmd_idx %d %d\n", __func__, cmd_idx, elvss_cmd.cmds[0].payload[0]);

	return &elvss_cmd;

end :
	pr_err("%s error", __func__);
	return NULL;
}

static struct dsi_panel_cmds *mdss_gamma(struct mdss_dsi_ctrl_pdata *ctrl, int *level_key)
{
	struct samsung_display_driver_data *vdd = check_valid_ctrl(ctrl);

	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: Invalid data ctrl : 0x%zx vdd : 0x%zx", __func__, (size_t)ctrl, (size_t)vdd);
		return NULL;
	}

	vdd->candela_level = get_candela_value(vdd, ctrl->ndx);

	pr_debug("%s bl_level : %d candela : %dCD\n", __func__, vdd->bl_level, vdd->candela_level);

	if (IS_ERR_OR_NULL(vdd->smart_dimming_dsi[ctrl->ndx]->generate_gamma)) {
		pr_err("%s generate_gamma is NULL error", __func__);
		return NULL;
	} else {
		vdd->smart_dimming_dsi[ctrl->ndx]->generate_gamma(
			vdd->smart_dimming_dsi[ctrl->ndx],
			vdd->candela_level,
			&vdd->dtsi_data[ctrl->ndx].gamma_tx_cmds[vdd->panel_revision].cmds[0].payload[1]);

		*level_key = PANEL_LEVE1_KEY;

		return &vdd->dtsi_data[ctrl->ndx].gamma_tx_cmds[vdd->panel_revision];
	}
}

static void dsi_update_mdnie_data(void)
{
	/* Update mdnie command */
	mdnie_data.DSI0_COLOR_BLIND_MDNIE_2 = DSI0_COLOR_BLIND_MDNIE_2;
	mdnie_data.DSI0_RGB_SENSOR_MDNIE_1 = NULL;
	mdnie_data.DSI0_RGB_SENSOR_MDNIE_2 = NULL;
	mdnie_data.DSI0_UI_DYNAMIC_MDNIE_2 = DSI0_UI_DYNAMIC_MDNIE_2;
	mdnie_data.DSI0_UI_STANDARD_MDNIE_2 = DSI0_UI_STANDARD_MDNIE_2;
	mdnie_data.DSI0_UI_AUTO_MDNIE_2 = DSI0_UI_AUTO_MDNIE_2;
	mdnie_data.DSI0_VIDEO_DYNAMIC_MDNIE_2 = DSI0_VIDEO_DYNAMIC_MDNIE_2;
	mdnie_data.DSI0_VIDEO_STANDARD_MDNIE_2 = DSI0_VIDEO_STANDARD_MDNIE_2;
	mdnie_data.DSI0_VIDEO_AUTO_MDNIE_2 = DSI0_VIDEO_AUTO_MDNIE_2;
	mdnie_data.DSI0_CAMERA_MDNIE_2 = DSI0_CAMERA_MDNIE_2;
	mdnie_data.DSI0_CAMERA_AUTO_MDNIE_2 = DSI0_CAMERA_AUTO_MDNIE_2;
	mdnie_data.DSI0_GALLERY_DYNAMIC_MDNIE_2 = DSI0_GALLERY_DYNAMIC_MDNIE_2;
	mdnie_data.DSI0_GALLERY_STANDARD_MDNIE_2 = DSI0_GALLERY_STANDARD_MDNIE_2;
	mdnie_data.DSI0_GALLERY_AUTO_MDNIE_2 = DSI0_GALLERY_AUTO_MDNIE_2;
	mdnie_data.DSI0_VT_DYNAMIC_MDNIE_2 = DSI0_VT_DYNAMIC_MDNIE_2;
	mdnie_data.DSI0_VT_STANDARD_MDNIE_2 = DSI0_VT_STANDARD_MDNIE_2;
	mdnie_data.DSI0_VT_AUTO_MDNIE_2 = DSI0_VT_AUTO_MDNIE_2;
	mdnie_data.DSI0_BROWSER_DYNAMIC_MDNIE_2 = DSI0_BROWSER_DYNAMIC_MDNIE_2;
	mdnie_data.DSI0_BROWSER_STANDARD_MDNIE_2 = DSI0_BROWSER_STANDARD_MDNIE_2;
	mdnie_data.DSI0_BROWSER_AUTO_MDNIE_2 = DSI0_BROWSER_AUTO_MDNIE_2;
	mdnie_data.DSI0_EBOOK_DYNAMIC_MDNIE_2 = DSI0_EBOOK_DYNAMIC_MDNIE_2;
	mdnie_data.DSI0_EBOOK_STANDARD_MDNIE_2 = DSI0_EBOOK_STANDARD_MDNIE_2;
	mdnie_data.DSI0_EBOOK_AUTO_MDNIE_2 = DSI0_EBOOK_AUTO_MDNIE_2;
	mdnie_data.DSI0_TDMB_DYNAMIC_MDNIE_2 = DSI0_UI_DYNAMIC_MDNIE_2;
	mdnie_data.DSI0_TDMB_STANDARD_MDNIE_2 = DSI0_UI_STANDARD_MDNIE_2;
	mdnie_data.DSI0_TDMB_AUTO_MDNIE_2 = DSI0_UI_AUTO_MDNIE_2;

	mdnie_data.DSI0_BYPASS_MDNIE = NULL;
	mdnie_data.DSI0_NEGATIVE_MDNIE = DSI0_NEGATIVE_MDNIE;
	mdnie_data.DSI0_COLOR_BLIND_MDNIE = DSI0_COLOR_BLIND_MDNIE;
	mdnie_data.DSI0_HBM_CE_MDNIE = NULL;
	mdnie_data.DSI0_HBM_CE_TEXT_MDNIE = NULL;
	mdnie_data.DSI0_RGB_SENSOR_MDNIE = NULL;
	mdnie_data.DSI0_CURTAIN = NULL;
	mdnie_data.DSI0_GRAYSCALE_MDNIE = NULL;
	mdnie_data.DSI0_GRAYSCALE_NEGATIVE_MDNIE = NULL;
	mdnie_data.DSI0_UI_DYNAMIC_MDNIE = DSI0_UI_DYNAMIC_MDNIE;
	mdnie_data.DSI0_UI_STANDARD_MDNIE = DSI0_UI_STANDARD_MDNIE;
	mdnie_data.DSI0_UI_NATURAL_MDNIE = DSI0_UI_NATURAL_MDNIE;
	mdnie_data.DSI0_UI_MOVIE_MDNIE = DSI0_UI_MOVIE_MDNIE;
	mdnie_data.DSI0_UI_AUTO_MDNIE = DSI0_UI_AUTO_MDNIE;
	mdnie_data.DSI0_VIDEO_OUTDOOR_MDNIE = NULL;
	mdnie_data.DSI0_VIDEO_DYNAMIC_MDNIE = DSI0_VIDEO_DYNAMIC_MDNIE;
	mdnie_data.DSI0_VIDEO_STANDARD_MDNIE = DSI0_VIDEO_STANDARD_MDNIE;
	mdnie_data.DSI0_VIDEO_NATURAL_MDNIE = DSI0_VIDEO_NATURAL_MDNIE;
	mdnie_data.DSI0_VIDEO_MOVIE_MDNIE = DSI0_VIDEO_MOVIE_MDNIE;
	mdnie_data.DSI0_VIDEO_AUTO_MDNIE = DSI0_VIDEO_AUTO_MDNIE;
	mdnie_data.DSI0_VIDEO_WARM_OUTDOOR_MDNIE = NULL;
	mdnie_data.DSI0_VIDEO_WARM_MDNIE = NULL;
	mdnie_data.DSI0_VIDEO_COLD_OUTDOOR_MDNIE = NULL;
	mdnie_data.DSI0_VIDEO_COLD_MDNIE = NULL;
	mdnie_data.DSI0_CAMERA_OUTDOOR_MDNIE = NULL;
	mdnie_data.DSI0_CAMERA_MDNIE = DSI0_CAMERA_MDNIE;
	mdnie_data.DSI0_CAMERA_AUTO_MDNIE = DSI0_CAMERA_AUTO_MDNIE;
	mdnie_data.DSI0_GALLERY_DYNAMIC_MDNIE = DSI0_GALLERY_DYNAMIC_MDNIE;
	mdnie_data.DSI0_GALLERY_STANDARD_MDNIE = DSI0_GALLERY_STANDARD_MDNIE;
	mdnie_data.DSI0_GALLERY_NATURAL_MDNIE = DSI0_GALLERY_NATURAL_MDNIE;
	mdnie_data.DSI0_GALLERY_MOVIE_MDNIE = DSI0_GALLERY_MOVIE_MDNIE;
	mdnie_data.DSI0_GALLERY_AUTO_MDNIE = DSI0_GALLERY_AUTO_MDNIE;
	mdnie_data.DSI0_VT_DYNAMIC_MDNIE = DSI0_VT_DYNAMIC_MDNIE;
	mdnie_data.DSI0_VT_STANDARD_MDNIE = DSI0_VT_STANDARD_MDNIE;
	mdnie_data.DSI0_VT_NATURAL_MDNIE = DSI0_VT_NATURAL_MDNIE;
	mdnie_data.DSI0_VT_MOVIE_MDNIE = DSI0_VT_MOVIE_MDNIE;
	mdnie_data.DSI0_VT_AUTO_MDNIE = DSI0_VT_AUTO_MDNIE;
	mdnie_data.DSI0_BROWSER_DYNAMIC_MDNIE = DSI0_BROWSER_DYNAMIC_MDNIE;
	mdnie_data.DSI0_BROWSER_STANDARD_MDNIE = DSI0_BROWSER_STANDARD_MDNIE;
	mdnie_data.DSI0_BROWSER_NATURAL_MDNIE = DSI0_BROWSER_NATURAL_MDNIE;
	mdnie_data.DSI0_BROWSER_MOVIE_MDNIE = DSI0_BROWSER_MOVIE_MDNIE;
	mdnie_data.DSI0_BROWSER_AUTO_MDNIE = DSI0_BROWSER_AUTO_MDNIE;
	mdnie_data.DSI0_EBOOK_DYNAMIC_MDNIE = DSI0_EBOOK_DYNAMIC_MDNIE;
	mdnie_data.DSI0_EBOOK_STANDARD_MDNIE = DSI0_EBOOK_STANDARD_MDNIE;
	mdnie_data.DSI0_EBOOK_NATURAL_MDNIE = DSI0_EBOOK_NATURAL_MDNIE;
	mdnie_data.DSI0_EBOOK_MOVIE_MDNIE = DSI0_EBOOK_MOVIE_MDNIE;
	mdnie_data.DSI0_EBOOK_AUTO_MDNIE = DSI0_EBOOK_AUTO_MDNIE;
	mdnie_data.DSI0_EMAIL_AUTO_MDNIE = DSI0_EMAIL_AUTO_MDNIE;
	mdnie_data.DSI0_TDMB_DYNAMIC_MDNIE = DSI0_UI_DYNAMIC_MDNIE;
	mdnie_data.DSI0_TDMB_STANDARD_MDNIE = DSI0_UI_STANDARD_MDNIE;
	mdnie_data.DSI0_TDMB_NATURAL_MDNIE = DSI0_UI_NATURAL_MDNIE;
	mdnie_data.DSI0_TDMB_MOVIE_MDNIE = DSI0_UI_MOVIE_MDNIE;
	mdnie_data.DSI0_TDMB_AUTO_MDNIE = DSI0_UI_AUTO_MDNIE;

	mdnie_data.mdnie_tune_value_dsi0 = mdnie_tune_value_dsi0;

	/* Update MDNIE data related with size, offset or index */
	/* NOTE: common mdnie code uses dsi0_bypass_mdnie_size as the
	 * command count of the mdnie tune packet */
	mdnie_data.dsi0_bypass_mdnie_size = ARRAY_SIZE(DSI0_UI_STANDARD_MDNIE);
	mdnie_data.mdnie_color_blinde_cmd_offset = MDNIE_COLOR_BLINDE_CMD_OFFSET;
	mdnie_data.mdnie_step_index[MDNIE_STEP1] = MDNIE_STEP1_INDEX;
	mdnie_data.mdnie_step_index[MDNIE_STEP2] = MDNIE_STEP2_INDEX;
	mdnie_data.address_scr_white[ADDRESS_SCR_WHITE_RED_OFFSET] = ADDRESS_SCR_WHITE_RED;
	mdnie_data.address_scr_white[ADDRESS_SCR_WHITE_GREEN_OFFSET] = ADDRESS_SCR_WHITE_GREEN;
	mdnie_data.address_scr_white[ADDRESS_SCR_WHITE_BLUE_OFFSET] = ADDRESS_SCR_WHITE_BLUE;
	mdnie_data.dsi0_rgb_sensor_mdnie_1_size = 0;
	mdnie_data.dsi0_rgb_sensor_mdnie_2_size = 0;
}

static void mdss_panel_init(struct samsung_display_driver_data *vdd)
{
	pr_info("%s\n", __func__);

	/*
	 * The heat bootloader does not pass lcd_id on the kernel command
	 * line, so get_lcd_attached("GET") would always return 0 and
	 * mdss_panel_attach_set() would keep the panel detached, blocking
	 * every panel command. This is a single panel device, so set the
	 * attach id explicitly (the stock driver did the same).
	 */
	get_lcd_attached("1");

	vdd->support_panel_max = S6E88A0_SS_WVGA_SUPPORT_PANEL_COUNT;

	vdd->support_mdnie_lite = true;

	vdd->mdnie_tune_size1 = 5;
	vdd->mdnie_tune_size2 = 108;

	/* ON/OFF */
	vdd->panel_func.samsung_panel_on_pre = mdss_panel_on_pre;
	vdd->panel_func.samsung_panel_on_post = mdss_panel_on_post;
	vdd->panel_func.samsung_panel_off_pre = NULL;
	vdd->panel_func.samsung_panel_off_post = NULL;

	/* DDI RX */
	vdd->panel_func.samsung_panel_revision = mdss_panel_revision;
	vdd->panel_func.samsung_manufacture_date_read = mdss_manufacture_date_read;
	vdd->panel_func.samsung_ddi_id_read = NULL;
	vdd->panel_func.samsung_hbm_read = NULL;
	vdd->panel_func.samsung_mdnie_read = mdss_mdnie_read;
	vdd->panel_func.samsung_smart_dimming_init = mdss_samart_dimming_init;
	vdd->panel_func.samsung_smart_get_conf = smart_get_conf_S6E88A0_SS_WVGA;

	/* Brightness */
	vdd->panel_func.samsung_brightness_tft_pwm_ldi = NULL;
	vdd->panel_func.samsung_brightness_hbm_off = NULL;
	vdd->panel_func.samsung_brightness_aid = mdss_aid;
	vdd->panel_func.samsung_brightness_acl_on = NULL;
	vdd->panel_func.samsung_brightness_acl_percent = mdss_acl_percent;
	vdd->panel_func.samsung_brightness_acl_off = mdss_acl_off;
	vdd->panel_func.samsung_brightness_elvss = mdss_elvss;
	vdd->panel_func.samsung_brightness_elvss_temperature1 = NULL;
	vdd->panel_func.samsung_brightness_elvss_temperature2 = NULL;
	vdd->panel_func.samsung_brightness_vint = NULL;
	vdd->panel_func.samsung_brightness_gamma = mdss_gamma;
	vdd->brightness[0].brightness_packet_tx_cmds_dsi.link_state = DSI_HS_MODE;

	/* HBM */
	vdd->panel_func.samsung_hbm_gamma = NULL;
	vdd->panel_func.samsung_hbm_etc = NULL;

	dsi_update_mdnie_data();
}

static int __init samsung_panel_init(void)
{
	struct samsung_display_driver_data *vdd = samsung_get_vdd();
	char panel_string[] = "qcom,mdss_dsi_samsung_wvga_video";
	char panel_string_common[] = "ss_dsi_panel_S6E88A0_SS_WVGA";
	char panel_string_lk[] = "6288A0_WVGA_video";

	vdd->panel_name = mdss_mdp_panel + 8;
	pr_info("%s : %s\n", __func__, vdd->panel_name);

	if (strstr(vdd->panel_name, panel_string_lk) ||
		!strncmp(vdd->panel_name, panel_string, strlen(panel_string)) ||
		!strncmp(vdd->panel_name, panel_string_common, strlen(panel_string_common)))
		vdd->panel_func.samsung_panel_init = mdss_panel_init;

	return 0;
}
early_initcall(samsung_panel_init);
