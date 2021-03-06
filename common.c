#include "common.h"
#include "priv.h"
#include <limits.h>

static void
safe_free (void *ptr)
{
  if (ptr)
    free (ptr);
}

fsdp_description_t *
fsdp_description_new (void)
{
  fsdp_description_t *result = malloc (sizeof (fsdp_description_t));

  result->version = 0;
  result->o_username = result->o_session_id =
    result->o_announcement_version = NULL;
  result->o_network_type = FSDP_NETWORK_TYPE_UNDEFINED;
  result->o_address_type = FSDP_ADDRESS_TYPE_UNDEFINED;
  result->o_address = NULL;
  result->s_name = NULL;
  result->i_information = NULL;
  result->u_uri = NULL;
  result->emails = NULL;
  result->emails_count = 0;
  result->phones = NULL;
  result->phones_count = 0;
  /* At first, there is no session-level definition for these
     parameters */
  result->c_network_type = FSDP_NETWORK_TYPE_UNDEFINED;
  result->c_address_type = FSDP_ADDRESS_TYPE_UNDEFINED;
  result->c_address.address = NULL;
  /* there is no session-level definition for these parameters */
  result->bw_modifiers = NULL;
  result->bw_modifiers_count = 0;
  result->time_periods = NULL;
  result->time_periods_count = 0;
  result->timezone_adj = NULL;
  result->k_encryption_method = FSDP_ENCRYPTION_METHOD_UNDEFINED;
  result->k_encryption_content = NULL;
  /* Default/undefined values for attributes */
  result->a_category = result->a_keywords = result->a_tool = NULL;
  result->a_type = FSDP_SESSION_TYPE_UNDEFINED;
  result->a_sendrecv_mode = FSDP_SENDRECV_UNDEFINED;
  result->a_charset = NULL;
  result->a_sdplangs = result->a_langs = NULL;
  result->a_controls = NULL;
  result->a_range = NULL;
  result->a_rtpmaps = NULL;
  result->a_rtpmaps_count = 0;
  result->a_sdplangs_count = 0;
  result->a_langs_count = 0;
  result->a_controls_count = 0;
  result->unidentified_attributes = NULL;
  result->unidentified_attributes_count = 0;
  result->media_announcements = NULL;
  result->media_announcements_count = 0;

  return result;
}

void
fsdp_description_delete (fsdp_description_t * dsc)
{
  fsdp_description_recycle (dsc);
  safe_free (dsc);
}

void
fsdp_description_recycle (fsdp_description_t * dsc)
{
  /* Recursively free all strings and arrays */
  unsigned int i, j;

  if (!dsc)
    return;

  safe_free (dsc->o_username);
  safe_free (dsc->o_session_id);
  safe_free (dsc->o_announcement_version);
  safe_free (dsc->o_address);
  safe_free (dsc->s_name);
  safe_free (dsc->i_information);
  safe_free (dsc->u_uri);

  for (i = 0; i < dsc->emails_count; i++)
    safe_free ((char *) dsc->emails[i]);
  safe_free (dsc->emails);

  for (i = 0; i < dsc->phones_count; i++)
    safe_free ((char *) dsc->phones[i]);
  safe_free (dsc->phones);

  safe_free (dsc->c_address.address);

  for (i = 0; i < dsc->bw_modifiers_count; i++)
    safe_free (dsc->bw_modifiers[i].b_unknown_bw_modt);
  safe_free (dsc->bw_modifiers);

  for (i = 0; i < dsc->time_periods_count; i++)
  {
    for (j = 0; j < dsc->time_periods[i]->repeats_count; j++)
    {
      safe_free (dsc->time_periods[i]->repeats[j]->offsets);
      safe_free (dsc->time_periods[i]->repeats[j]);
    }
    safe_free (dsc->time_periods[i]->repeats);
    safe_free (dsc->time_periods[i]);
  }
  safe_free (dsc->time_periods);

  safe_free (dsc->timezone_adj);
  safe_free (dsc->a_category);
  safe_free (dsc->a_keywords);
  safe_free (dsc->a_tool);

  for (i = 0; i < dsc->a_rtpmaps_count; i++)
    safe_free (dsc->a_rtpmaps[i]);
  safe_free (dsc->a_rtpmaps);

  safe_free (dsc->a_charset);

  for (i = 0; i < dsc->a_sdplangs_count; i++)
    safe_free (dsc->a_sdplangs[i]);
  safe_free (dsc->a_sdplangs);

  for (i = 0; i < dsc->a_langs_count; i++)
    safe_free (dsc->a_langs[i]);
  safe_free (dsc->a_langs);

  for (i = 0; i < dsc->a_controls_count; i++)
    safe_free (dsc->a_controls[i]);
  safe_free (dsc->a_controls);

  safe_free (dsc->a_range);

  for (i = 0; i < dsc->media_announcements_count; i++)
  {
    if (!dsc->media_announcements[i]) continue;
    for (j = 0; j < dsc->media_announcements[i]->formats_count; j++)
      safe_free (dsc->media_announcements[i]->formats[j]);
    safe_free (dsc->media_announcements[i]->formats);
    safe_free (dsc->media_announcements[i]->i_title);

    for (j = 0; j < dsc->media_announcements[i]->bw_modifiers_count; j++)
    {
      if (FSDP_BW_MOD_TYPE_UNKNOWN ==
          dsc->media_announcements[i]->bw_modifiers[j].b_mod_type)
        safe_free (dsc->media_announcements[i]->bw_modifiers[j].
                   b_unknown_bw_modt);
    }
    safe_free (dsc->media_announcements[i]->bw_modifiers);

    safe_free (dsc->media_announcements[i]->k_encryption_content);

    for (j = 0; j < dsc->media_announcements[i]->a_rtpmaps_count; j++)
    {
      safe_free (dsc->media_announcements[i]->a_rtpmaps[j]->pt);
      safe_free (dsc->media_announcements[i]->a_rtpmaps[j]->
                 encoding_name);
      safe_free (dsc->media_announcements[i]->a_rtpmaps[j]->parameters);
      safe_free (dsc->media_announcements[i]->a_rtpmaps[j]);
    }
    safe_free (dsc->media_announcements[i]->a_rtpmaps);

    for (j = 0; j < dsc->media_announcements[i]->a_sdplangs_count; j++)
      safe_free (dsc->media_announcements[i]->a_sdplangs[j]);
    safe_free (dsc->media_announcements[i]->a_sdplangs);

    for (j = 0; j < dsc->media_announcements[i]->a_langs_count; j++)
      safe_free (dsc->media_announcements[i]->a_langs[j]);
    safe_free (dsc->media_announcements[i]->a_langs);

    for (j = 0; j < dsc->media_announcements[i]->a_controls_count; j++)
      safe_free (dsc->media_announcements[i]->a_controls[j]);
    safe_free (dsc->media_announcements[i]->a_controls);

    for (j = 0; j < dsc->media_announcements[i]->a_fmtps_count; j++)
      safe_free (dsc->media_announcements[i]->a_fmtps[j]);
    safe_free (dsc->media_announcements[i]->a_fmtps);

    for (j = 0;
         j < dsc->media_announcements[i]->unidentified_attributes_count; j++)
      safe_free (dsc->media_announcements[i]->unidentified_attributes[j]);
    safe_free (dsc->media_announcements[i]->unidentified_attributes);
    safe_free (dsc->media_announcements[i]);
  }
  safe_free (dsc->media_announcements);

  /* This prevents the user to make the library crash when incorrectly
     using recycled but not rebuilt descriptions */
  dsc->emails_count = 0;
  dsc->phones_count = 0;
  dsc->bw_modifiers_count = 0;
  dsc->time_periods_count = 0;
  dsc->media_announcements_count = 0;
}


char *base64_encode(char *out, int out_size, const uint8_t *in, int in_size)
{
	static const char b64[] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	char *ret, *dst;
	unsigned i_bits = 0;
	int i_shift = 0;
	int bytes_remaining = in_size;

	if (in_size >= UINT_MAX / 4 ||
		out_size < BASE64_SIZE(in_size))
		return NULL;
	ret = dst = out;
	while (bytes_remaining > 3) {
		i_bits = RB32(in);
		in += 3; bytes_remaining -= 3;
		*dst++ = b64[ i_bits>>26		];
		*dst++ = b64[(i_bits>>20) & 0x3F];
		*dst++ = b64[(i_bits>>14) & 0x3F];
		*dst++ = b64[(i_bits>>8 ) & 0x3F];
	}
	i_bits = 0;
	while (bytes_remaining) {
		i_bits = (i_bits << 8) + *in++;
		bytes_remaining--;
		i_shift += 8;
	}
	while (i_shift > 0) {
		*dst++ = b64[(i_bits << 6 >> i_shift) & 0x3f];
		i_shift -= 6;
	}
	while ((dst - ret) & 3)
		*dst++ = '=';
	*dst = '\0';

	return ret;
}
